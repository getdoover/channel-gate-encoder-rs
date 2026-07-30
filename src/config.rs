//! Deployment config — a port of `src/channel_gate_encoder/app_config.py`.
//!
//! The config **keys** are the contract with an already-deployed
//! `app_config.json`, and pydoover derives each key from the element's display
//! name. So every field here is named exactly what the Python schema exported
//! (`channel_a_pin`, `mm_per_count`, `display_refresh_period_s`, …) and carries
//! the same title, description, default and bounds. `cargo run -- export`
//! regenerates `doover_config.json` from this struct.

use doover::Config;

/// Config for the channel gate quadrature encoder.
///
/// Two proximity sensors (channels A and B) act as a quadrature encoder on a
/// toothed target driven by the gate. Only the sensors' RISING edges are
/// captured — they are always a quarter target-pitch (90 deg) apart — so
/// decoding is 2x: two counts per tooth cycle, with direction inferred from
/// which of the two A/B rise gaps is the short one. The app publishes the gate
/// height. Because the encoder is *incremental*, the height is only trustworthy
/// once homed against the limit switch (or a manual zero), so the app re-zeros
/// every time the home switch is seen.
#[derive(Debug, Clone, Config)]
pub struct ChannelGateEncoderConfig {
    // --- Encoder inputs ----------------------------------------------------
    /// Digital input wired to proximity sensor A.
    #[config(min = 0)]
    pub channel_a_pin: i64,

    /// Digital input wired to proximity sensor B (mounted a quarter
    /// target-pitch from A so it lags/leads A by ~90 degrees).
    #[config(min = 0)]
    pub channel_b_pin: i64,

    /// Gate travel per decoded count, which is also the position granularity.
    /// Only RISING edges are captured, one per channel per tooth, so decoding
    /// is 2x (2 counts per full target-tooth cycle) and this is target pitch /
    /// 2. e.g. a 20 mm tooth pitch -> 10.0. This is DOUBLE the old both-edge
    /// (4x) value for the same target -- halve the tooth pitch if you need the
    /// resolution back.
    #[config(title = "mm per Count", default = 2.0, min = 0)]
    pub mm_per_count: f64,

    /// Teeth on the target wheel (rising edges per channel per revolution).
    /// Decoding is 2x, so counts_per_rev = 2 x this -- used for the revolutions
    /// and RPM readouts.
    #[config(default = 16, min = 1)]
    pub pulses_per_rev: i64,

    /// Flip the counting direction so the gate rising reads as an increasing
    /// height. Toggle this instead of swapping the sensors.
    #[config(default = false)]
    pub invert_direction: bool,

    /// Hardware debounce pushed to both channel DI pins. Keep small -- too high
    /// drops counts when the gate moves quickly.
    #[config(title = "Debounce (ms)", default = 2, min = 0)]
    pub debounce_ms: i64,

    /// Read channel A/B edges by polling the platform's batched DI event log
    /// instead of per-edge streaming callbacks. On real Doovit hardware this
    /// is the ONLY viable quadrature path, and even it cannot read direction
    /// from timing alone: the IO firmware harvests edges in ~50 ms sweeps and
    /// emits them grouped by pin, so cross-channel order/timing is destroyed
    /// at any real gate speed (measured 2026-07-30 on doovit-0bb070: arrival
    /// alternation 43%, fw-timestamp phase IQR spanning the ambiguity band).
    /// Direction must come from the controller via `/direction_hint`, which
    /// only the POLL path consults (the stream path passes hint=0). Streaming
    /// remains for sims that deliver per-edge in true time order.
    #[config(default = true)]
    pub use_event_polling: bool,

    /// Take the count's SIGN exclusively from the controller's
    /// `/direction_hint` pushes and never from A/B rise timing. On real Doovit
    /// hardware the firmware's ~50 ms harvest destroys cross-channel edge
    /// timing (see `use_event_polling`), so timing inference produces garbage
    /// direction; with this set, edges under an active hint are signed by the
    /// commanded direction, and edges with no hint are held (unsigned) or
    /// carried on the last commanded sign (ambiguous) — both visible in the
    /// diagnostics. Pair with `use_event_polling = false` for per-edge
    /// responsiveness. Leave false only on platforms whose pulse delivery
    /// preserves true edge order (the sim).
    #[config(default = false)]
    pub hint_only_direction: bool,

    /// How often the DI event log is polled in event-polling mode. The firmware
    /// buffer holds ~600 events; poll fast enough that it never fills between
    /// polls at your top edge rate.
    #[config(title = "Event Poll Period (s)", default = 0.25, min = 0.05)]
    pub event_poll_period_s: f64,

    /// Write DO0/1/2 off once at app start. The platform restores
    /// digital-output states across reboots, so after a crash mid-motion the
    /// outputs come back energised unless something asserts them off. Enable on
    /// any install where this app is the safety backstop (e.g. bench rigs).
    #[config(default = false)]
    pub assert_outputs_off_on_start: bool,

    // --- Homing / zero reference -------------------------------------------
    /// Digital input for the end-of-travel limit switch that establishes the
    /// zero. Leave unset to home only via the button.
    #[config(min = 0)]
    pub home_switch_pin: Option<i64>,

    /// Set if the limit switch reads LOW when the gate is at the home position
    /// (falling edge triggers homing).
    #[config(default = false)]
    pub home_switch_active_low: bool,

    /// The gate height when the home switch is active. Usually 0 for a switch
    /// at fully closed, or the full travel for one at fully open.
    #[config(title = "Home Height (mm)", default = 0.0)]
    pub home_height_mm: f64,

    // --- Gate geometry / display -------------------------------------------
    /// Full travel of the gate, used for the percent-open readout and range
    /// colouring.
    #[config(title = "Gate Travel (mm)", default = 1000.0, min = 0)]
    pub gate_travel_mm: f64,

    /// How often the height/status readouts refresh. Counting is event-driven
    /// and unaffected by this.
    #[config(title = "Display Refresh Period (s)", default = 0.5, min = 0.1)]
    pub display_refresh_period_s: f64,

    /// How often absolute position and direction are written to tags. Counting
    /// is event-driven and completely decoupled from this: a pulse callback only
    /// bumps an in-memory count, and this timer derives and publishes the
    /// result. Lower it for tighter closed-loop control (the controller cannot
    /// stop the gate more accurately than the position it can see -- overshoot
    /// is roughly gate speed x this interval), raise it to cut cloud traffic on
    /// a slow gate.
    #[config(title = "Tag Publish Interval (s)", default = 0.5, min = 0.05)]
    pub tag_publish_interval_s: f64,
}

impl ChannelGateEncoderConfig {
    /// Whether a limit switch is wired (pydoover `homing_enabled`).
    pub fn homing_enabled(&self) -> bool {
        self.home_switch_pin.is_some()
    }
}
