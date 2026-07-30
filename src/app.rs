//! The [`Application`] impl — a port of
//! `src/channel_gate_encoder/application.py`.
//!
//! All the interesting work lives in [`EncoderCore`]; this layer does three
//! things and nothing else:
//!
//! 1. `setup` — configure the DI pins for rising interrupts, restore a
//!    persisted count, start the ingest listeners, arm homing, serve the debug
//!    endpoint.
//! 2. `main_loop` — the **publish timer**. The runner ticks it at
//!    `min(display_refresh_period_s, tag_publish_interval_s)` and this gates on
//!    `tag_publish_interval_s`, then writes the derived snapshot to tags. The
//!    gate has to live here rather than in a task of its own because the
//!    framework flushes tag writes in exactly one place — `commit_tags()`, once
//!    per main-loop pass (`doover/src/docker/application.rs:773-776`) — so a
//!    detached publish task would still only reach the cloud at the main loop's
//!    cadence.
//! 3. `on_ui_command` — the two field buttons.

use std::time::Duration;

use doover::docker::{DiConfigUpdate, Edge};
use doover::error::Result;
use doover::{AppContext, Application, PlatformClient, UiCommand};

use crate::config::ChannelGateEncoderConfig;
use crate::core::{EncoderCore, Params};
use crate::debug_server::{self, DebugHandles};
use crate::state::unix_secs;
use crate::tags::ChannelGateEncoderTags;
use crate::ui::ChannelGateEncoderUi;

pub struct ChannelGateEncoder {
    config: ChannelGateEncoderConfig,
    tags: ChannelGateEncoderTags,
    ui: ChannelGateEncoderUi,
    core: EncoderCore,
    plt: Option<PlatformClient>,
}

impl ChannelGateEncoder {
    /// Safety backstop for installs where nothing else asserts the outputs: the
    /// FIRMWARE restores digital-output states from flash across reboots, so
    /// after a crash mid-motion DO0/1/2 come back energised. At boot this app
    /// often starts BEFORE the platform interface accepts connections (observed
    /// live: a one-shot version failed with connection-refused while the
    /// restored outputs ran a motor unsupervised), so the guard RETRIES in the
    /// background until one write verifiably succeeds — a readback of all-off,
    /// not merely an accepted write.
    fn spawn_boot_safety_guard(plt: PlatformClient) {
        tokio::spawn(async move {
            for attempt in 0..150u32 {
                let write = plt.set_dos(&[0, 1, 2], &[false, false, false]).await;
                if write.is_ok() {
                    match plt.fetch_dos(&[0, 1, 2]).await {
                        Ok(readback) if readback.len() == 3 && !readback.iter().any(|v| *v) => {
                            tracing::warn!(
                                "Boot-safety guard: DO0/1/2 confirmed off (attempt {})",
                                attempt + 1
                            );
                            return;
                        }
                        Ok(readback) => {
                            tracing::error!("Boot-safety guard: readback not clean: {readback:?}")
                        }
                        Err(e) if attempt % 15 == 0 => {
                            tracing::warn!("Boot-safety guard retrying ({e})")
                        }
                        Err(_) => {}
                    }
                } else if attempt % 15 == 0 {
                    tracing::warn!("Boot-safety guard retrying ({:?})", write.err());
                }
                tokio::time::sleep(Duration::from_secs(2)).await;
            }
            tracing::error!("Boot-safety guard GAVE UP after 5 minutes - check the platform");
        });
    }
}

#[doover::async_trait]
impl Application for ChannelGateEncoder {
    type Config = ChannelGateEncoderConfig;
    type Tags = ChannelGateEncoderTags;
    type Ui = ChannelGateEncoderUi;

    fn create(
        config: ChannelGateEncoderConfig,
        tags: ChannelGateEncoderTags,
        ui: ChannelGateEncoderUi,
    ) -> Self {
        let core = EncoderCore::new(Params::from_config(&config));
        Self { config, tags, ui, core, plt: None }
    }

    fn ui(&self) -> Option<&ChannelGateEncoderUi> {
        Some(&self.ui)
    }

    fn ui_mut(&mut self) -> Option<&mut ChannelGateEncoderUi> {
        Some(&mut self.ui)
    }

    /// Tick at least as often as the faster of the two consumers (the display
    /// refresh and the tag publish timer), then let each gate itself. Tag
    /// publishing can never be faster than the loop that flushes it.
    fn loop_target_period(&self) -> Duration {
        let display = self.config.display_refresh_period_s.max(0.01);
        let publish = self.config.tag_publish_interval_s.max(0.01);
        Duration::from_secs_f64(display.min(publish))
    }

    async fn setup(&mut self, _ctx: &AppContext) -> Result<()> {
        let plt_uri = std::env::var("PLT_URI").unwrap_or_else(|_| "127.0.0.1:50053".to_string());
        tracing::info!("connecting to platform interface at {plt_uri}");
        let plt = PlatformClient::connect(format!("http://{plt_uri}")).await?;

        if self.config.assert_outputs_off_on_start {
            Self::spawn_boot_safety_guard(plt.clone());
        }

        // Configure both channel pins for RISING interrupts and a small hardware
        // debounce. irq_edge matters well beyond bus load: the IO firmware's
        // harvest majority-votes every edge its ISR saw in a 50 ms window into
        // ONE event, so an ISR that also sees falling edges can report a tooth
        // as a falling edge and destroy the rising one. Asking the firmware for
        // rising-only is the only way to keep falling edges out of that vote.
        let di_config = DiConfigUpdate {
            irq_edge: Some(Edge::Rising),
            debounce_ms: Some(self.config.debounce_ms as i32),
            ..Default::default()
        };
        for pin in [self.core.params.a_pin, self.core.params.b_pin] {
            if let Err(e) = plt.set_di_config(pin, &di_config).await {
                tracing::warn!("set_di_config({pin}, irq_edge=rising) failed: {e}");
            }
        }

        // Restore a persisted count across a brief restart. Only trustworthy if
        // the gate didn't move while the app was down; the home switch re-zeros
        // it on the next pass regardless.
        if let Some(persisted) = self.tags.RawCount.get() {
            if persisted != 0 {
                self.core.state.lock().expect("state lock").decoder.count = persisted;
            }
        }
        if let Some(homed) = self.tags.Homed.get() {
            self.core.state.lock().expect("state lock").homed = homed;
        }

        // Channels A and B, rising edge of each -> 2x decoding. Two ingest
        // modes:
        //   poll   -- read the platform's batched DI event log on a timer and
        //             decode from it. Survives high edge rates; per-edge
        //             streaming is what wedges the IO firmware (hard watchdog
        //             reset) once the wheel spins fast (~100+ edges/s).
        //   stream -- per-edge callbacks. This is the path the fidelity soak
        //             measures, and the one the 500 ms publish timer was
        //             designed around.
        if self.config.use_event_polling {
            self.core.start_event_polling(&plt).await;
        } else {
            self.core.start_pulse_listeners(&plt);
        }

        // Home / limit switch (optional).
        if let Some(home_pin) = self.core.params.home_pin {
            let home_cfg = DiConfigUpdate { debounce_ms: Some(20), ..Default::default() };
            if let Err(e) = plt.set_di_config(home_pin, &home_cfg).await {
                tracing::debug!("set_di_config(home {home_pin}) failed: {e}");
            }
            self.core.start_home_listeners(&plt, home_pin);
            self.core.seed_home_from_level(&plt, home_pin).await;
        } else {
            tracing::info!("No home switch configured - homing is manual only");
        }

        // Baseline the movement tracking on the restored height, so the first
        // publish after a restart doesn't read a huge phantom speed/direction,
        // and seed the debug snapshot with real values (notably a non-zero
        // counts_per_rev) so a poll before the first main_loop never serves
        // placeholders.
        self.core.baseline(unix_secs());

        // Range colouring depends on the configured travel, which `UiBuild`
        // can't see (Python `ChannelGateEncoderUI.setup`).
        let travel =
            if self.config.gate_travel_mm > 0.0 { self.config.gate_travel_mm } else { 1000.0 };
        self.ui.height.ranges = Some(vec![
            doover::ui::Range::new("Closed", 0, travel * 0.05, doover::ui::Colour::BLUE),
            doover::ui::Range::new(
                "Part Open",
                travel * 0.05,
                travel * 0.95,
                doover::ui::Colour::GREEN,
            ),
            doover::ui::Range::new("Open", travel * 0.95, travel, doover::ui::Colour::YELLOW),
        ]);

        debug_server::spawn(
            debug_server::PORT,
            DebugHandles {
                snapshot: self.core.snapshot.clone(),
                state: self.core.state.clone(),
                hints: self.core.hints.clone(),
            },
        )
        .await;

        self.plt = Some(plt);
        Ok(())
    }

    async fn main_loop(&mut self, _ctx: &AppContext) -> Result<()> {
        // The pulse callbacks only touch an in-memory count; this is where that
        // count becomes an absolute position and reaches the outside world. The
        // publish rate is config-driven and independent of the pulse rate, so a
        // 30 Hz edge stream still only writes tags every tag_publish_interval_s.
        let now = unix_secs();
        if !self.core.due_to_publish(now) {
            return Ok(());
        }
        let snap = self.core.publish(now);

        self.tags.Height.set(snap.height_mm).await?;
        self.tags.PercentOpen.set(snap.percent_open).await?;
        self.tags.RawCount.set(snap.count).await?;
        self.tags.Homed.set(snap.homed).await?;
        self.tags.Direction.set(snap.direction.to_string()).await?;
        self.tags.Speed.set(snap.speed_mm_min).await?;
        self.tags.HomeSwitch.set(snap.home_switch).await?;
        self.tags.MissedEdges.set(snap.missed_edges as i64).await?;
        self.tags.AmbiguousEdges.set(snap.ambiguous_edges as i64).await?;
        self.tags.Heartbeat.set(snap.ts).await?;
        self.tags.Revolutions.set(snap.revolutions).await?;
        self.tags.RPM.set(snap.rpm).await?;
        self.tags.RotationDirection.set(snap.rotation_direction.to_string()).await?;

        // Warn until the encoder has been homed (height is unverified before
        // then).
        self.ui.not_homed_warning.interaction.element.hidden =
            Some(serde_json::Value::Bool(snap.homed));
        Ok(())
    }

    async fn on_ui_command(&mut self, _ctx: &AppContext, cmd: &UiCommand) -> Result<()> {
        if cmd.is(&self.ui.set_home) {
            // Manually zero the encoder at the configured home height.
            self.core.home_now();
        } else if cmd.is(&self.ui.reset_missed) {
            self.core.reset_missed();
        }
        Ok(())
    }

    async fn on_shutdown(&mut self, _ctx: &AppContext) -> Result<()> {
        self.core.shutdown();
        if let Some(plt) = &self.plt {
            plt.close();
        }
        Ok(())
    }
}
