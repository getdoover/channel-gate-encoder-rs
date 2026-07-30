//! The in-memory state the pulse callbacks touch, and the snapshot the publish
//! timer produces.
//!
//! The split is the whole point of the design (Python
//! `application.py:347 _on_edge` vs `:421 _publish_state`): **a pulse callback
//! only mutates [`EncoderState`]**, which is a `Mutex` lock, an integer bump
//! and a few float comparisons. Deriving height / percent / direction / rpm and
//! writing tags happens on the publish timer, at `tag_publish_interval_s`,
//! completely decoupled from the edge rate.

use std::sync::LazyLock;
use std::time::Instant;

use serde_json::{json, Value};

use crate::quadrature::{Channel, RisingEdgeDecoder};

/// Process start, so [`monotonic_secs`] is a cheap f64 monotonic clock — the
/// Rust equivalent of Python's `time.monotonic()`.
static EPOCH: LazyLock<Instant> = LazyLock::new(Instant::now);

/// Seconds since process start, monotonic. Used for every edge timestamp: the
/// direction decode measures a 16.7 ms gap, so it must never see a clock that
/// can step.
pub fn monotonic_secs() -> f64 {
    EPOCH.elapsed().as_secs_f64()
}

/// Wall-clock seconds since the unix epoch (for `Heartbeat` and the speed
/// window, which are reported to humans rather than used for decode).
pub fn unix_secs() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64())
        .unwrap_or(0.0)
}

/// Everything a pulse callback is allowed to touch.
#[derive(Debug)]
pub struct EncoderState {
    pub decoder: RisingEdgeDecoder,
    /// Pulse callbacks actually delivered, across both channels. The fidelity
    /// criterion is `injected == callbacks`.
    pub callbacks: u64,
    /// Callbacks whose `DiPulse.value` was `true`.
    ///
    /// This is a **live probe for the upstream bug**: the platform interface
    /// never sets `value` on `pulseCounterResponse`
    /// (`doover-platform-interface/src/doover_platform_interface/platform_iface_base.py:268-281`),
    /// so on every driver — real hardware included — it arrives as the proto3
    /// default and doover-rs surfaces it as `Some(false)`/`None`. This counter
    /// therefore reads 0 forever. If it is ever non-zero on a device, upstream
    /// has been fixed and the level could start being trusted.
    pub pulses_reporting_high: u64,
    /// Whether the home/limit switch is currently asserted.
    pub home_active: bool,
    /// Whether the count has ever been zeroed against the reference.
    pub homed: bool,
    /// Bumped on every home, so the publish path can notice one happened in a
    /// callback and drop the rate history (a count discontinuity must not read
    /// as a speed spike).
    pub home_epoch: u64,

    /// Optional arrival trace: `(channel, monotonic_secs)` for every pulse
    /// callback, appended in the callback itself.
    ///
    /// Off by default (`None`, one branch in the hot path). Switched on by
    /// [`enable_trace`](Self::enable_trace) so the fidelity soak can measure the
    /// **inter-edge gap error distribution** — the number that actually matters
    /// now that direction is a timing comparison: at 15 rising/s/sensor the
    /// short A->B gap is 16.7 ms and the discriminator flips at 33.3 ms, so
    /// host-side scheduling jitter above ~8 ms starts corrupting direction. The
    /// Python app measured a worst gap error of 14.4 ms against that 16.7 ms
    /// nominal, and intermittently landed inside the ambiguity band.
    pub trace: Option<Vec<(Channel, f64)>>,

    /// Times a `startPulseCounter` stream ended and was re-established. Each one
    /// costs a count's worth of direction information on that pin (the firmware
    /// reports no period for the first edge of a new stream), so a growing value
    /// means the position should be re-homed.
    pub stream_reconnects: u64,

    // --- polled-ingest bookkeeping (only used with use_event_polling) ---
    pub events_decoded: u64,
    pub event_cursor: i64,
    pub poll_errors: u64,
}

impl EncoderState {
    pub fn new(invert: bool) -> Self {
        Self {
            decoder: RisingEdgeDecoder::new(invert),
            callbacks: 0,
            pulses_reporting_high: 0,
            home_active: false,
            homed: false,
            home_epoch: 0,
            trace: None,
            stream_reconnects: 0,
            events_decoded: 0,
            event_cursor: 0,
            poll_errors: 0,
        }
    }

    /// Start recording pulse arrival times (see [`trace`](Self::trace)).
    /// `capacity` pre-allocates so the hot path never reallocates mid-soak.
    pub fn enable_trace(&mut self, capacity: usize) {
        self.trace = Some(Vec::with_capacity(capacity));
    }

    /// Re-zero the encoder against the limit switch (or manual button).
    pub fn home(&mut self) {
        self.decoder.zero();
        self.homed = true;
        self.home_epoch += 1;
    }
}

/// Exactly the numbers the app last published, served verbatim by the debug
/// endpoint (Python `application.py:463 _last_state`).
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub ts: f64,
    pub count: i64,
    pub revolutions: f64,
    pub rpm: f64,
    pub rotation_direction: &'static str,
    pub height_mm: f64,
    pub percent_open: f64,
    pub direction: &'static str,
    pub speed_mm_min: f64,
    pub homed: bool,
    pub home_switch: bool,
    pub missed_edges: u64,
    /// Counts signed by a HELD direction rather than a measured one. A non-zero
    /// and growing value means the A/B rise timing is not readable, so the SIGN
    /// of the position is not trustworthy even though no edges were lost.
    pub ambiguous_edges: u64,
    /// Edges held out of the count because no direction has been measured yet.
    /// Non-zero only before the first cycle of a run completes.
    pub unsigned_edges: u64,
    /// Falling edges that still arrived despite `irq_edge="rising"`.
    pub filtered_edges: u64,
    /// Edges whose period came from the firmware's PIO measurement.
    pub hw_period_edges: u64,
    /// Edges where the firmware reported `dt_secs = 0` (first edge of a pin, or
    /// after a dropped transition). Growing during steady motion is the
    /// app-visible proxy for a firmware debounce-FIFO overflow.
    pub hw_period_missing: u64,
    /// Edges where the hardware and host periods disagreed by more than 10% — a
    /// direct measurement of host-side delivery jitter.
    pub period_disagreements: u64,
    /// Pulse-stream reconnects; each costs one edge's direction information.
    pub stream_reconnects: u64,
    pub counts_per_rev: i64,
    pub uptime_s: f64,
    pub input_mode: &'static str,
    pub edge_mode: &'static str,
    pub pulse_callbacks: u64,
    pub pulses_reporting_high: u64,
    pub events_decoded: u64,
    pub event_cursor: i64,
    pub poll_errors: u64,
    /// Instantaneous sign of the last decoded step, straight off the two
    /// sensors (+1 opening / -1 closing / 0 nothing decoded yet).
    pub pulse_direction: i8,
    pub publish_interval_s: f64,
}

impl Default for Snapshot {
    fn default() -> Self {
        Self {
            ts: 0.0,
            count: 0,
            revolutions: 0.0,
            rpm: 0.0,
            rotation_direction: "stopped",
            height_mm: 0.0,
            percent_open: 0.0,
            direction: "stopped",
            speed_mm_min: 0.0,
            homed: false,
            home_switch: false,
            missed_edges: 0,
            ambiguous_edges: 0,
            unsigned_edges: 0,
            filtered_edges: 0,
            hw_period_edges: 0,
            hw_period_missing: 0,
            period_disagreements: 0,
            stream_reconnects: 0,
            counts_per_rev: 0,
            uptime_s: 0.0,
            input_mode: "stream",
            edge_mode: "rising_2x",
            pulse_callbacks: 0,
            pulses_reporting_high: 0,
            events_decoded: 0,
            event_cursor: 0,
            poll_errors: 0,
            pulse_direction: 0,
            publish_interval_s: 0.5,
        }
    }
}

/// Round to `places` decimals, the way the Python app's `round(x, n)` calls do
/// for the published values.
fn round_to(value: f64, places: u32) -> f64 {
    let f = 10f64.powi(places as i32);
    (value * f).round() / f
}

impl Snapshot {
    /// The JSON body the debug endpoint serves — same keys as the Python
    /// `/state` response, so the bench motor harness needs no changes.
    pub fn to_json(&self) -> Value {
        json!({
            "ts": round_to(self.ts, 3),
            "count": self.count,
            "revolutions": round_to(self.revolutions, 4),
            "rpm": round_to(self.rpm, 1),
            "rotation_direction": self.rotation_direction,
            "height_mm": round_to(self.height_mm, 2),
            "percent_open": round_to(self.percent_open, 2),
            "direction": self.direction,
            "speed_mm_min": round_to(self.speed_mm_min, 1),
            "homed": self.homed,
            "home_switch": self.home_switch,
            "missed_edges": self.missed_edges,
            "ambiguous_edges": self.ambiguous_edges,
            "unsigned_edges": self.unsigned_edges,
            "filtered_edges": self.filtered_edges,
            "hw_period_edges": self.hw_period_edges,
            "hw_period_missing": self.hw_period_missing,
            "period_disagreements": self.period_disagreements,
            "stream_reconnects": self.stream_reconnects,
            "counts_per_rev": self.counts_per_rev,
            "uptime_s": round_to(self.uptime_s, 1),
            "input_mode": self.input_mode,
            "edge_mode": self.edge_mode,
            "pulse_callbacks": self.pulse_callbacks,
            "pulses_reporting_high": self.pulses_reporting_high,
            "events_decoded": self.events_decoded,
            "event_cursor": self.event_cursor,
            "poll_errors": self.poll_errors,
            "pulse_direction": self.pulse_direction,
            "publish_interval_s": self.publish_interval_s,
        })
    }
}
