//! [`EncoderCore`] — the hardware-facing half of the app: the rising-edge
//! subscriptions, the optional polled-event ingest, and the publish-timer
//! derivation.
//!
//! Deliberately free of [`AppContext`](doover::AppContext) so the integration
//! tests can drive it against a gRPC platform double without a device agent,
//! while the [`Application`](doover::Application) impl in [`crate::app`] does
//! nothing but ferry a [`Snapshot`] into tags.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use doover::docker::Edge;
use doover::error::Result;
use doover::PlatformClient;
use tokio::task::AbortHandle;
use tokio_stream::StreamExt;

use crate::config::ChannelGateEncoderConfig;
use crate::quadrature::{
    counts_per_rev, merge_new_edge_events, revolutions, rotation_direction, travel_direction,
    Channel, RateEstimator, RawDiEvent, RPM_STOPPED_THRESHOLD,
};
use crate::state::{monotonic_secs, unix_secs, EncoderState, Snapshot};

/// Plain values pulled out of the deployment config once, so the pulse path and
/// the publish path never touch the typed config (and so tests can build a core
/// without one).
#[derive(Debug, Clone)]
pub struct Params {
    pub a_pin: i32,
    pub b_pin: i32,
    pub home_pin: Option<i32>,
    /// `true` when the switch reads HIGH at home (`!home_switch_active_low`).
    pub home_active_high: bool,
    pub mm_per_count: f64,
    pub pulses_per_rev: i64,
    pub home_height_mm: f64,
    pub gate_travel_mm: f64,
    pub publish_interval_s: f64,
    pub debounce_ms: i32,
    pub use_event_polling: bool,
    pub event_poll_period_s: f64,
    pub invert_direction: bool,
}

impl Params {
    pub fn from_config(cfg: &ChannelGateEncoderConfig) -> Self {
        Self {
            a_pin: cfg.channel_a_pin as i32,
            b_pin: cfg.channel_b_pin as i32,
            home_pin: cfg.home_switch_pin.map(|p| p as i32),
            home_active_high: !cfg.home_switch_active_low,
            mm_per_count: cfg.mm_per_count,
            pulses_per_rev: cfg.pulses_per_rev,
            home_height_mm: cfg.home_height_mm,
            gate_travel_mm: cfg.gate_travel_mm,
            publish_interval_s: cfg.tag_publish_interval_s,
            debounce_ms: cfg.debounce_ms as i32,
            use_event_polling: cfg.use_event_polling,
            event_poll_period_s: cfg.event_poll_period_s,
            invert_direction: cfg.invert_direction,
        }
    }

    /// A minimal set for tests: two pins, streaming ingest, no homing.
    pub fn for_test(a_pin: i32, b_pin: i32) -> Self {
        Self {
            a_pin,
            b_pin,
            home_pin: None,
            home_active_high: true,
            mm_per_count: 2.0,
            pulses_per_rev: 16,
            home_height_mm: 0.0,
            gate_travel_mm: 1000.0,
            publish_interval_s: 0.5,
            debounce_ms: 2,
            use_event_polling: false,
            event_poll_period_s: 0.25,
            invert_direction: false,
        }
    }
}

pub struct EncoderCore {
    pub params: Params,
    /// Shared with the pulse callbacks, the poll task and the debug endpoint.
    pub state: Arc<Mutex<EncoderState>>,
    /// The last published snapshot, served verbatim by the debug endpoint.
    pub snapshot: Arc<Mutex<Snapshot>>,
    /// External direction authority: a TIMELINE of `(epoch_ms, +1/-1/0)` hint
    /// changes pushed by whatever commands the motion. Polled events are signed
    /// by the hint active at their CAPTURE time (the platform syncs events
    /// 30-90 s late, so decode-time hints mis-sign stragglers). Seeded neutral
    /// — with no hint the decoder infers direction from the A/B rise timing
    /// alone, which is the design intent.
    pub hints: Arc<Mutex<Vec<(i64, i8)>>>,

    rate: RateEstimator,
    last_publish: f64,
    last_count: i64,
    last_height: f64,
    last_time: f64,
    last_home_epoch: u64,
    input_mode: &'static str,
    start: f64,
    tasks: Vec<AbortHandle>,
}

impl EncoderCore {
    pub fn new(params: Params) -> Self {
        let state = EncoderState::new(params.invert_direction);
        let mut core = Self {
            state: Arc::new(Mutex::new(state)),
            snapshot: Arc::new(Mutex::new(Snapshot::default())),
            hints: Arc::new(Mutex::new(vec![(0, 0)])),
            rate: RateEstimator::default(),
            last_publish: 0.0,
            last_count: 0,
            last_height: 0.0,
            last_time: unix_secs(),
            last_home_epoch: 0,
            input_mode: if params.use_event_polling { "poll" } else { "stream" },
            start: monotonic_secs(),
            tasks: Vec::new(),
            params,
        };
        core.last_height = core.home_height_for(0);
        core
    }

    fn locked(&self) -> std::sync::MutexGuard<'_, EncoderState> {
        self.state.lock().expect("encoder state lock poisoned")
    }

    // ------------------------------------------------------------------
    // Pulse-stream ingest
    // ------------------------------------------------------------------

    /// Subscribe ONE rising-edge listener per channel (Python
    /// `application.py:312 _init_pulse_streaming`).
    ///
    /// Rising-only is the requirement, and it happens to also side-step a
    /// platform bug that made the previous both-edge design awkward: **the
    /// interface never populates the pulse level.**
    /// `platform_iface_base.py:275-281` yields
    /// `pulseCounterResponse(response_header=…, di=di, dt_secs=dt_secs)` and the
    /// proto declares `optional bool value`, so doover-rs sees `value: None` and
    /// `PulseCounterUpdate::value` is `false` on EVERY pulse, on every driver,
    /// real hardware included. Here it simply does not matter: **the channel is
    /// bound from which subscription delivered the event** — captured by the
    /// closure below — so there is no level to read. `pulses_reporting_high`
    /// keeps score of the bug on-device.
    ///
    /// This consumes the raw `startPulseCounter` stream (via
    /// [`PlatformClient::subscribe_di_pulses`], see [`Self::spawn_raw_listener`])
    /// rather than `get_new_pulse_counter`, which matters: doover-rs *did* port
    /// pydoover's 0.2 s pulse grace period, but only inside
    /// `PulseCounter::start_listener_pulses` (`docker/platform.rs:1034`), where
    /// it is a private `const` with no setter — a Rust app cannot zero it the
    /// way the Python app does (`counter.pulse_grace_period = 0.0`). The
    /// low-level listener has no grace period at all, so taking that path is
    /// the only way to keep the first 0.2 s of travel (6 counts at 30 rising
    /// edges/s).
    ///
    /// The direction decode needs two intervals, and they come from different
    /// places:
    ///
    /// * **`period`** (same channel, one full tooth cycle) comes off the wire.
    ///   In rising-only mode `doovit_fw` 1.9.1 defines `dt_secs` as exactly that
    ///   and measures it **in the PIO state machine, with no CPU-tick jitter**
    ///   (`dio.py:300-304`). Using it means the discriminator's threshold no
    ///   longer depends on when the host got round to running this callback — see
    ///   [`RisingEdgeDecoder::edge_with_period`].
    /// * **`gap`** (cross-channel, the 16.7 ms 90-degree spacing) has no hardware
    ///   equivalent on the live pulse stream — `dt_secs` is per-pin — so the
    ///   callback takes [`monotonic_secs`] itself and that term keeps whatever
    ///   scheduling jitter the runtime added. The two periods are cross-checked,
    ///   so that jitter shows up in `period_disagreements` rather than silently
    ///   eroding the margin.
    pub fn start_pulse_listeners(&mut self, plt: &PlatformClient) {
        for (channel, pin) in [(Channel::A, self.params.a_pin), (Channel::B, self.params.b_pin)] {
            self.spawn_pulse_listener(plt, channel, pin);
        }
        tracing::info!(
            "Rising-edge listeners started on A=DI{}, B=DI{} (one per pin)",
            self.params.a_pin,
            self.params.b_pin
        );
    }

    /// One rising-edge listener, consuming the raw `startPulseCounter` stream and
    /// reconnecting when it ends.
    ///
    /// This deliberately does **not** use
    /// [`PlatformClient::start_di_pulse_listener`], which would be the obvious
    /// choice, because that helper carries pydoover's `dt_secs > 0` filter
    /// (`doover/src/docker/platform.rs:850-852`, *"pydoover only counts pulses
    /// with dt > 0"*) — and on this firmware **`dt_secs == 0.0` is a legitimate
    /// pulse**. `doovit_fw` 1.9.1 emits `dt_out = 0.0` whenever the PIO has no
    /// period to report (`dio.py:304`), which is:
    ///
    /// * the **first edge of a pin** — one lost count per channel per stream
    ///   connect, so once per restart and once per reconnect; and
    /// * **after a dropped transition** (a debounce-FIFO overflow the firmware
    ///   recovered from) — i.e. precisely when the count is already at risk.
    ///
    /// So the helper silently loses exactly the counts that matter most. Consuming
    /// the stream directly avoids it, at the cost of owning the reconnect loop.
    ///
    /// Distinguishing "no period" from "not a pulse" is why this reads
    /// `DiPulse::dt_secs` as an `Option` rather than unwrapping it: the platform
    /// opens every stream with a header-only frame that sets `di` but leaves
    /// `dt_secs` **absent** (`platform_iface_base.py:268-271`), whereas a real
    /// pulse always sets it, even to `0.0`. `None` = not a pulse, `Some(_)` =
    /// a pulse. proto3 explicit presence makes that distinction survive the wire.
    fn spawn_pulse_listener(&mut self, plt: &PlatformClient, channel: Channel, pin: i32) {
        let state = Arc::clone(&self.state);
        self.spawn_raw_listener(plt, pin, Edge::Rising, move |hw_period, reports_high| {
            // Timestamp taken before the lock: it is the delivery time and carries
            // whatever scheduling jitter the runtime added — at 16.7 ms A-to-B
            // spacing that is a real risk, which is what the soak measures.
            let t = monotonic_secs();
            // The whole body of the pulse path: at 30 rising edges/s combined it
            // must only mutate cheap in-memory state. Deriving height/direction
            // and publishing tags is the publish timer's job (see `publish`).
            let mut st = state.lock().expect("encoder state lock poisoned");
            st.callbacks += 1;
            if reports_high {
                st.pulses_reporting_high += 1;
            }
            if let Some(trace) = &mut st.trace {
                trace.push((channel, t));
            }
            st.decoder.edge_with_period(channel, t, 0, hw_period);
        });
    }

    /// Consume the raw `startPulseCounter` stream for one (pin, edge), calling
    /// `on_pulse(hw_period_secs, reports_high)` for every pulse and reconnecting
    /// when the stream ends.
    ///
    /// This exists instead of [`PlatformClient::start_di_pulse_listener`] because
    /// that helper carries pydoover's `dt_secs > 0` filter
    /// (`doover/src/docker/platform.rs:850-852`, *"pydoover only counts pulses
    /// with dt > 0"*) — and on this firmware **`dt_secs == 0.0` is a legitimate
    /// pulse**. `doovit_fw` 1.9.1 emits `dt_out = 0.0` whenever the PIO has no
    /// period to report (`dio.py:304`), which is:
    ///
    /// * the **first edge of a pin** — so one lost count per channel per stream
    ///   connect, i.e. once per restart and once per reconnect; and
    /// * **after a dropped transition** (a debounce-FIFO overflow the firmware
    ///   recovered from) — precisely when the count is already at risk.
    ///
    /// On the home switch the same filter is worse than a lost count: the first
    /// edge on that pin is *the gate first reaching home*, and dropping it means
    /// the very first homing event of a boot never fires.
    ///
    /// Distinguishing "no period" from "not a pulse" is why this reads
    /// `DiPulse::dt_secs` as an `Option` rather than unwrapping it: the platform
    /// opens every stream with a header-only frame that sets `di` but leaves
    /// `dt_secs` **absent** (`platform_iface_base.py:268-271`), whereas a real
    /// pulse always sets it, even to `0.0`. `None` = not a pulse, `Some(_)` = a
    /// pulse; proto3 explicit presence makes that distinction survive the wire.
    fn spawn_raw_listener<F>(&mut self, plt: &PlatformClient, pin: i32, edge: Edge, on_pulse: F)
    where
        F: Fn(f64, bool) + Send + Sync + 'static,
    {
        let plt = plt.clone();
        let state = Arc::clone(&self.state);
        let task = tokio::spawn(async move {
            loop {
                match plt.subscribe_di_pulses(pin, edge).await {
                    Ok(stream) => {
                        let mut stream = std::pin::pin!(stream);
                        while let Some(item) = stream.next().await {
                            match item {
                                Ok(pulse) => {
                                    // Absent dt_secs = the stream's opening
                                    // header frame, not an edge.
                                    let Some(hw_period) = pulse.dt_secs else { continue };
                                    on_pulse(
                                        (hw_period as f64).max(0.0),
                                        pulse.value == Some(true),
                                    );
                                }
                                Err(e) => {
                                    tracing::error!("error receiving pulse for di={pin}: {e}");
                                    break;
                                }
                            }
                        }
                        tracing::info!("pulseCounter for di={pin} ended; reconnecting");
                    }
                    Err(e) => {
                        tracing::error!("error subscribing to pulses for di={pin}: {e}")
                    }
                }
                // A reconnect costs this pin one edge's worth of direction
                // information: the firmware reports no period for the first edge
                // of a new stream. Homing clears it.
                state.lock().expect("encoder state lock poisoned").stream_reconnects += 1;
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        });
        self.tasks.push(task.abort_handle());
    }

    /// Watch the home/limit switch with **two** subscriptions — rising and
    /// falling — and take the level from whichever one fired.
    ///
    /// The Python app subscribes once with `edge="both"` and reads `di_value`
    /// (`application.py:361 _on_home`), which the platform never populates: with
    /// `home_switch_active_low=false` that comparison is `0 == 1` on every edge
    /// and **homing never triggers**; with `active_low=true` it fires on both
    /// polarities. Binding the level from the subscription is the same fix the
    /// channel decode uses, applied to the switch.
    pub fn start_home_listeners(&mut self, plt: &PlatformClient, pin: i32) {
        for (edge, level) in [(Edge::Rising, true), (Edge::Falling, false)] {
            let state = Arc::clone(&self.state);
            let active_high = self.params.home_active_high;
            self.spawn_raw_listener(plt, pin, edge, move |_hw_period, _reports_high| {
                let active = level == active_high;
                let mut st = state.lock().expect("encoder state lock poisoned");
                if active && !st.home_active {
                    st.home();
                }
                st.home_active = active;
            });
        }
        tracing::info!("Home switch listeners (rising + falling) started on DI{pin}");
    }

    /// Read the switch's current level and home immediately if it is asserted —
    /// a gate parked ON the switch at boot never produces an edge.
    pub async fn seed_home_from_level(&mut self, plt: &PlatformClient, pin: i32) {
        match plt.fetch_di(pin).await {
            Ok(level) => {
                let active = level == self.params.home_active_high;
                let mut st = self.locked();
                st.home_active = active;
                if active {
                    st.home();
                }
            }
            Err(e) => tracing::warn!("Could not read initial home switch level: {e}"),
        }
    }

    // ------------------------------------------------------------------
    // Polled-event ingest (the Python app's default mode)
    // ------------------------------------------------------------------

    /// Baseline the event cursor at "now" and start the poll task (Python
    /// `application.py:248 _init_event_polling`).
    ///
    /// History in the platform's event log predates this boot of the app (the
    /// wheel may have moved while we were down), so it is discarded: the count
    /// starts wherever the shaft is and homing/zero establishes the reference.
    pub async fn start_event_polling(&mut self, plt: &PlatformClient) {
        match fetch_edge_batches(plt, self.params.a_pin, self.params.b_pin, 0).await {
            Ok((edges, newest)) => {
                self.locked().event_cursor = newest;
                tracing::info!(
                    "Event polling baseline: cursor={newest} ({} stale events discarded)",
                    edges.len()
                );
            }
            // Start from 0: the first poll will then discard history itself via
            // the same path (worst case it decodes pre-boot motion once, which
            // a subsequent zero/home corrects).
            Err(e) => tracing::warn!("Event polling baseline failed ({e}); cursor=0"),
        }

        let plt = plt.clone();
        let state = Arc::clone(&self.state);
        let hints = Arc::clone(&self.hints);
        let (a_pin, b_pin) = (self.params.a_pin, self.params.b_pin);
        let period = Duration::from_secs_f64(self.params.event_poll_period_s);
        let task = tokio::spawn(async move {
            loop {
                let cursor = state.lock().expect("state lock").event_cursor;
                match fetch_edge_batches(&plt, a_pin, b_pin, cursor).await {
                    Ok((edges, newest)) => {
                        let timeline = hints.lock().expect("hint lock").clone();
                        let mut st = state.lock().expect("state lock");
                        st.decoder.feed_batch(&edges, |t_ms| hint_at(&timeline, t_ms));
                        st.event_cursor = newest;
                        st.events_decoded += edges.len() as u64;
                    }
                    Err(e) => {
                        let mut st = state.lock().expect("state lock");
                        st.poll_errors += 1;
                        tracing::warn!("DI event poll failed (#{}): {e}", st.poll_errors);
                    }
                }
                tokio::time::sleep(period).await;
            }
        });
        self.tasks.push(task.abort_handle());
        tracing::info!(
            "Quadrature EVENT POLLING started on A=DI{a_pin}, B=DI{b_pin} every {:.2}s",
            self.params.event_poll_period_s
        );
    }

    // ------------------------------------------------------------------
    // Derived values
    // ------------------------------------------------------------------

    /// Decoded counts per wheel revolution: 2x, one per channel's rise.
    pub fn counts_per_rev(&self) -> i64 {
        counts_per_rev(self.params.pulses_per_rev)
    }

    fn home_height_for(&self, count: i64) -> f64 {
        self.params.home_height_mm + count as f64 * self.params.mm_per_count
    }

    /// Absolute gate height in mm for the current count.
    pub fn height_mm(&self) -> f64 {
        self.home_height_for(self.locked().decoder.count)
    }

    fn percent_open(&self, height_mm: f64) -> f64 {
        if self.params.gate_travel_mm <= 0.0 {
            return 0.0;
        }
        (height_mm / self.params.gate_travel_mm * 100.0).clamp(0.0, 100.0)
    }

    /// Whether `tag_publish_interval_s` has elapsed since the last publish.
    pub fn due_to_publish(&self, now: f64) -> bool {
        (now - self.last_publish) >= self.params.publish_interval_s
    }

    /// Baseline the publish timer and movement tracking without emitting
    /// anything (called at the end of `setup`, so the first real publish after a
    /// restart doesn't read a huge phantom speed).
    pub fn baseline(&mut self, now: f64) {
        self.last_height = self.height_mm();
        self.last_time = now;
        let count = self.locked().decoder.count;
        self.last_count = count;
        let mut snap = self.derive(now);
        snap.ts = now;
        *self.snapshot.lock().expect("snapshot lock") = snap;
    }

    /// Derive the full published state from the current count. Pure apart from
    /// the rate window and the movement deltas it advances.
    pub fn publish(&mut self, now: f64) -> Snapshot {
        self.last_publish = now;
        let snap = self.derive(now);
        *self.snapshot.lock().expect("snapshot lock") = snap.clone();
        snap
    }

    fn derive(&mut self, now: f64) -> Snapshot {
        // ONE lock, ONE copy of everything, released before any arithmetic.
        // This has to be a single critical section: `count` and `home_epoch`
        // read under separate locks could straddle a home landing in a
        // callback, and the snapshot would then carry a pre-home count with a
        // post-home epoch. Nothing below re-reads the shared state.
        let read = {
            let st = self.locked();
            StateRead {
                count: st.decoder.count,
                missed: st.decoder.missed,
                ambiguous: st.decoder.ambiguous,
                unsigned: st.decoder.unsigned,
                filtered: st.decoder.filtered,
                hw_period_used: st.decoder.hw_period_used,
                hw_period_missing: st.decoder.hw_period_missing,
                period_disagreements: st.decoder.period_disagreements,
                stream_reconnects: st.stream_reconnects,
                pulse_direction: st.decoder.direction,
                homed: st.homed,
                home_switch: st.home_active,
                callbacks: st.callbacks,
                high: st.pulses_reporting_high,
                events: st.events_decoded,
                cursor: st.event_cursor,
                poll_errors: st.poll_errors,
                home_epoch: st.home_epoch,
            }
        };
        let StateRead {
            count,
            missed,
            ambiguous,
            unsigned,
            filtered,
            hw_period_used,
            hw_period_missing,
            period_disagreements,
            stream_reconnects,
            pulse_direction,
            homed,
            home_switch,
            callbacks,
            high,
            events,
            cursor,
            poll_errors,
            home_epoch,
        } = read;

        // A home since the last publish is a count discontinuity: drop the rate
        // history so it can't read as a phantom spike.
        if home_epoch != self.last_home_epoch {
            self.rate.reset();
            self.last_home_epoch = home_epoch;
            self.last_count = count;
            self.last_height = self.home_height_for(count);
        }

        let height = self.home_height_for(count);
        let percent = self.percent_open(height);

        // Speed from height change since the last publish.
        let dt = now - self.last_time;
        let speed = if dt > 0.0 { (height - self.last_height) / dt * 60.0 } else { 0.0 };
        self.last_height = height;
        self.last_time = now;

        // Direction comes from the PULSE COUNT, not from the speed:
        // mm_per_count may legitimately be 0/unset on a fresh install, which
        // would make a speed-derived direction read "stopped" during real
        // movement. Nothing here consults which valve is energised, so the
        // answer is the same however the gate is being driven.
        let count_delta = count - self.last_count;
        let direction = travel_direction(count_delta);
        self.last_count = count;

        let cpr = self.counts_per_rev();
        self.rate.add(now, count);
        let rpm = self.rate.rpm(cpr);

        Snapshot {
            ts: now,
            count,
            revolutions: revolutions(count, cpr),
            rpm,
            rotation_direction: rotation_direction(rpm, RPM_STOPPED_THRESHOLD),
            height_mm: height,
            percent_open: percent,
            direction,
            speed_mm_min: speed.abs(),
            homed,
            home_switch,
            missed_edges: missed,
            ambiguous_edges: ambiguous,
            unsigned_edges: unsigned,
            filtered_edges: filtered,
            hw_period_edges: hw_period_used,
            hw_period_missing,
            period_disagreements,
            stream_reconnects,
            counts_per_rev: cpr,
            uptime_s: monotonic_secs() - self.start,
            input_mode: self.input_mode,
            edge_mode: "rising_2x",
            pulse_callbacks: callbacks,
            pulses_reporting_high: high,
            events_decoded: events,
            event_cursor: cursor,
            poll_errors,
            pulse_direction,
            publish_interval_s: self.params.publish_interval_s,
        }
    }

    /// Manual zero (the `set_home` button, or `POST /zero`).
    pub fn home_now(&mut self) {
        self.locked().home();
        tracing::info!("Homed: count zeroed at home height {:.1} mm", self.params.home_height_mm);
    }

    /// Clear the missed-edge diagnostic counter (the `reset_missed` button).
    pub fn reset_missed(&mut self) {
        self.locked().decoder.missed = 0;
    }

    /// Cancel every background task this core started.
    pub fn shutdown(&mut self) {
        for task in self.tasks.drain(..) {
            task.abort();
        }
    }
}

impl Drop for EncoderCore {
    fn drop(&mut self) {
        self.shutdown();
    }
}

/// One atomic read of everything [`EncoderCore::derive`] needs out of the shared
/// state, so the whole snapshot comes from a single critical section.
struct StateRead {
    count: i64,
    missed: u64,
    ambiguous: u64,
    unsigned: u64,
    filtered: u64,
    hw_period_used: u64,
    hw_period_missing: u64,
    period_disagreements: u64,
    stream_reconnects: u64,
    pulse_direction: i8,
    homed: bool,
    home_switch: bool,
    callbacks: u64,
    high: u64,
    events: u64,
    cursor: i64,
    poll_errors: u64,
    home_epoch: u64,
}

/// One poll: fetch both pins' RISING event logs and merge the new edges.
///
/// `Edge::Rising` here mirrors the `irq_edge="rising"` pin config: any falling
/// event that still comes back means the pins were not reconfigured, and
/// `RisingEdgeDecoder::filtered` will say so rather than absorbing it.
async fn fetch_edge_batches(
    plt: &PlatformClient,
    a_pin: i32,
    b_pin: i32,
    last_id: i64,
) -> Result<(Vec<crate::quadrature::EdgeRecord>, i64)> {
    let (_, a_events) = plt.fetch_di_events(a_pin, Edge::Rising, false, 0).await?;
    let (_, b_events) = plt.fetch_di_events(b_pin, Edge::Rising, false, 0).await?;
    let a: Vec<RawDiEvent> = a_events.iter().map(to_raw).collect();
    let b: Vec<RawDiEvent> = b_events.iter().map(to_raw).collect();
    Ok(merge_new_edge_events(&[(Channel::A, a.as_slice()), (Channel::B, b.as_slice())], last_id))
}

fn to_raw(e: &doover::docker::PlatformEvent) -> RawDiEvent {
    RawDiEvent { event_id: e.event_id as i64, event: e.event.clone(), time_ms: e.time }
}

/// The direction hint that was active at capture time `t_ms`.
pub fn hint_at(timeline: &[(i64, i8)], t_ms: i64) -> i8 {
    let mut active = 0;
    for (ts, d) in timeline {
        if *ts <= t_ms {
            active = *d;
        } else {
            break;
        }
    }
    active
}

/// Record a direction-hint change, trimming entries older than 10 minutes but
/// always keeping the latest state.
pub fn push_hint(hints: &Mutex<Vec<(i64, i8)>>, hint: i8) {
    let now_ms = (unix_secs() * 1000.0) as i64;
    let mut timeline = hints.lock().expect("hint lock");
    timeline.push((now_ms, hint));
    let cutoff = now_ms - 600_000;
    while timeline.len() > 1 && timeline[1].0 < cutoff {
        timeline.remove(0);
    }
}
