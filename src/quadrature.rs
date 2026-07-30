//! Pure rising-edge decode logic — a port of the Python app's
//! `src/channel_gate_encoder/quadrature.py`.
//!
//! Two proximity sensors watch a toothed target, mounted a quarter-pitch apart
//! so their outputs sit ~90 deg out of phase (channels A and B). This app
//! captures **rising edges only** — one subscription per pin,
//! [`Edge::Rising`](doover::docker::Edge::Rising) — because that is all the
//! prox sensors are asked to deliver: A rises once per tooth, B rises once per
//! tooth, and the two rises are always a quarter cycle apart.
//!
//! What rising-only costs, stated plainly up front, because it drives every
//! design decision in this module:
//!
//! * Decoding is **2x**, not 4x: two counts per full tooth cycle (one per
//!   channel's rise), so `mm_per_count` is tooth pitch / 2 and the position
//!   granularity is half a tooth pitch.
//! * There is **no level information at all**. The classic Gray-code
//!   transition table needs the 2-bit (A, B) state, which requires seeing
//!   falling edges too, so that table is not here. This is doubly true on a
//!   real Doover platform, where the pulse stream never populates the level
//!   anyway (see [`crate::app`] and the notes on `DiPulse::value`).
//! * **Edge ordering alone cannot give direction.** Rising edges alternate
//!   A, B, A, B in *both* directions — reversing the target only swaps which of
//!   the two inter-channel gaps is the short one. Direction therefore comes
//!   from TIMING: driven one way the A-rise -> B-rise gap is a quarter cycle
//!   and B-rise -> A-rise is three quarters; driven the other way the short gap
//!   is B -> A. [`RisingEdgeDecoder`] documents the exact test, the ambiguity
//!   band, and the failure modes.
//! * A rising edge of A happens at a different *physical* target position
//!   depending on the travel direction (a quarter cycle before A's falling edge
//!   going one way, a quarter cycle after it going the other). Counting rising
//!   edges is therefore a position quantiser **with direction hysteresis**: see
//!   [`RisingEdgeDecoder::reversal_backlash_counts`].
//!
//! This module is deliberately free of any doover / hardware dependency so it
//! can be unit-tested in isolation (see `tests/quadrature.rs`). The application
//! layer feeds it rising edges from the platform (streamed callbacks or the
//! polled DI event log) and reads back a signed count. The same module also
//! provides a small pure [`RateEstimator`] plus revs/RPM helpers used to turn
//! that count into rotational speed for the readouts.

use std::collections::{BTreeMap, VecDeque};

/// Which proximity sensor an edge came from.
///
/// This is bound from **which subscription delivered the pulse**, never read
/// off the wire: the platform interface does not populate the pulse level (see
/// [`crate::app::EncoderCore::start_pulse_listeners`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Channel {
    A,
    B,
}

impl Channel {
    fn idx(self) -> usize {
        match self {
            Channel::A => 0,
            Channel::B => 1,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Channel::A => "A",
            Channel::B => "B",
        }
    }
}

/// One decoded edge from a polled DI event batch: `(event_id, channel, level,
/// time_ms)`. `level` is `true` for a rising edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeRecord {
    pub event_id: i64,
    pub channel: Channel,
    pub level: bool,
    pub time_ms: i64,
}

/// Incremental 2x decoder driven by RISING edges only.
///
/// Feed it every rising edge from the two channels, with the time it was
/// observed; it maintains a signed [`count`](Self::count) (position, in counts
/// of half a tooth cycle) relative to wherever it was last zeroed. Because it
/// is *incremental*, the count is only meaningful once the gate has been homed
/// against a known reference (limit switch or manual zero).
///
/// # How direction is decided
///
/// Rising edges arrive A, B, A, B whichever way the target turns, so the
/// arrival ORDER carries no direction information whatsoever. What does carry
/// it is the ratio of the two inter-channel gaps. With the sensors a quarter
/// pitch apart, one direction produces gaps of (1/4, 3/4) of a cycle and the
/// other (3/4, 1/4). So on each edge, using `period` = the interval since the
/// SAME channel last rose (one full cycle) and `gap` = the interval since the
/// OTHER channel rose:
///
/// * `2 * gap < period` — the gap just closed was the short one, so the other
///   channel led this one;
/// * `2 * gap > period` — this channel led, three quarters of a cycle ago.
///
/// "B's rise leads A's" is the positive direction (+1), matching the forward
/// Gray-code walk the platform simulators emit (`00 -> 01 -> 11 -> 10`, i.e. B
/// rises first) and what the app reports as "opening" before `invert_direction`
/// is applied. "A leads B" is -1. One edge with a usable `period` is enough, so
/// direction is known within one cycle of motion starting and does not need an
/// averaging window.
///
/// At the specified 15 rising edges/s per sensor the two gaps are **16.7 ms**
/// and **50.0 ms**, and the discriminator threshold sits at half a period
/// (33.3 ms) with the [`AMBIGUITY_BAND`](Self::AMBIGUITY_BAND) making anything
/// between 25.0 ms and 41.7 ms unreadable. That is the whole timing budget: any
/// transport that smears the A->B gap by more than ~8 ms starts producing
/// ambiguous counts, and one that smears it past 33 ms produces *confidently
/// wrong* ones.
///
/// # Failure modes — all counted, none hidden
///
/// * [`missed`](Self::missed) — two rises in a row on the same channel. That is
///   either a lost edge on the other channel or a reversal inside a quadrant,
///   and the two are indistinguishable from rising edges alone. The edge is
///   still counted (travel did happen) but signed by the held direction.
/// * [`ambiguous`](Self::ambiguous) — an edge signed by a *held* direction
///   rather than a measured one: a same-channel repeat, or a gap that landed
///   inside [`AMBIGUITY_BAND`](Self::AMBIGUITY_BAND) of half the cycle (which
///   means the waveform is not what a 90 deg mounting produces, or the
///   timestamps were mangled in transport). Every one of these is a count whose
///   sign may be wrong.
/// * [`unsigned`](Self::unsigned) — edges seen before ANY direction could be
///   measured — at most the first two of a run, since the third has a
///   same-channel period to work with. They are held out of `count` rather than
///   guessed at, then flushed with the real sign the moment one measurement
///   lands.
/// * Direction reversal — the cycle that contains the reversal has a corrupted
///   gap pattern, so 2 edges are signed with the OLD direction (measured: **4
///   counts** of position error, deterministic and rate-independent). Because
///   `count` integrates, that error does not heal itself — it stays in the
///   position until the next home. On top of that sits the unavoidable
///   [`reversal_backlash_counts`](Self::reversal_backlash_counts) trigger-point
///   shift.
#[derive(Debug, Clone)]
pub struct RisingEdgeDecoder {
    /// Signed position, in counts of half a tooth cycle, since the last zero.
    pub count: i64,
    /// Flip the sign of every step, so "gate rising" reads as an increasing
    /// count without physically re-wiring the two sensors.
    pub invert: bool,
    /// Raw sensor sense: +1 = B's rise leads A's, -1 = A leads B, 0 = unknown.
    pub sense: i8,
    /// `sense` with `invert` applied — the sign actually added to `count`, i.e.
    /// the direction read STRAIGHT OFF THE TWO SENSORS. It is correct no matter
    /// what is driving the gate: our own valves, a second controller, a hand
    /// crank, or gravity pulling it back down through a leaking valve. Nothing
    /// here consults the commanded direction, by design.
    pub direction: i8,
    /// Same-channel repeats: a lost edge on the other channel, or a reversal
    /// inside a quadrant.
    pub missed: u64,
    /// Counts signed by a held direction rather than a measured one.
    pub ambiguous: u64,
    /// Edges held out of `count` because no direction has been measured yet.
    pub unsigned: u64,
    /// Falling edges offered to [`feed_batch`](Self::feed_batch). Not a loss —
    /// this design does not use them — but worth surfacing if it is ever
    /// non-zero, since it means the pins are still configured `irq_edge="both"`.
    pub filtered: u64,

    /// Edges whose `period` came from the firmware's PIO measurement rather than
    /// from host arrival times. See [`edge_with_period`](Self::edge_with_period).
    pub hw_period_used: u64,
    /// Edges where the firmware offered no period (`dt_secs == 0`), so the host
    /// fallback was used.
    ///
    /// The firmware sets `dt_secs = None -> 0.0` on **the first edge(s) of a pin
    /// and after a dropped transition** (`doovit_fw` 1.9.1 `dio.py:300-304`). A
    /// handful at startup is normal; a *growing* count during steady motion is
    /// the app-visible proxy for the firmware's `dbc.dropped` debounce-FIFO
    /// overflow, which is otherwise only visible in the firmware log.
    pub hw_period_missing: u64,
    /// Edges where the hardware period and the host-computed period disagreed by
    /// more than [`PERIOD_DISAGREEMENT`](Self::PERIOD_DISAGREEMENT) of the
    /// hardware value.
    ///
    /// The hardware value is authoritative, so this is a **direct measurement of
    /// host-side delivery jitter** — the thing that makes a timing-based
    /// direction decode fragile. Non-zero means the host is late enough that the
    /// gap term (which has no hardware equivalent on the live pulse stream) is
    /// also suspect.
    pub period_disagreements: u64,

    last_ch: Option<Channel>,
    last_t: f64,
    /// Last rise time per channel, indexed by [`Channel::idx`].
    last_rise: [Option<f64>; 2],
}

impl RisingEdgeDecoder {
    /// A gap this close to half a cycle carries no direction information. The
    /// two inter-channel gaps should be a quarter and three quarters of a
    /// cycle, i.e. `2 * gap - period` should be +/- half a period; anything
    /// inside +/-1/8 cycle of the halfway mark is treated as unreadable. This
    /// tolerates sensor mountings from roughly 45 to 135 deg of phase.
    pub const AMBIGUITY_BAND: f64 = 0.25;

    /// Relative gap between the hardware and host periods above which
    /// [`period_disagreements`](Self::period_disagreements) is incremented. 10%
    /// of a 66.7 ms period is 6.7 ms — comfortably above normal delivery jitter,
    /// well below the 16.7 ms of gap error the discriminator can absorb.
    pub const PERIOD_DISAGREEMENT: f64 = 0.10;

    pub fn new(invert: bool) -> Self {
        Self {
            count: 0,
            invert,
            sense: 0,
            direction: 0,
            missed: 0,
            ambiguous: 0,
            unsigned: 0,
            filtered: 0,
            hw_period_used: 0,
            hw_period_missing: 0,
            period_disagreements: 0,
            last_ch: None,
            last_t: 0.0,
            last_rise: [None, None],
        }
    }

    /// Register one rising edge at time `t` (monotonic seconds) and return the
    /// applied step.
    ///
    /// Fully synchronous and non-blocking, so it is safe to call straight from
    /// interleaving pulse callbacks under one mutex — which is exactly what the
    /// application does, and the whole reason the pulse path is cheap.
    ///
    /// `hint` is an optional external direction authority (+1 / -1, 0 = none),
    /// typically whichever controller commanded the motion. When supplied it
    /// overrides the timing inference for this edge; the timing state is still
    /// updated so an unhinted edge later can measure normally.
    ///
    /// Every rising edge moves the count by exactly one, because a rising edge
    /// *is* half a tooth cycle of travel. Only its SIGN is in question, which
    /// is why the sign's provenance is tracked in `ambiguous` / `unsigned`.
    ///
    /// Returns 0 while no direction has been measured yet (the edge is held,
    /// see `unsigned`), and on the edge that first resolves the direction it
    /// returns the whole held run at once — so the return value is the applied
    /// count delta, not always +/-1.
    ///
    /// Equivalent to [`edge_with_period`](Self::edge_with_period) with no
    /// hardware period, i.e. `period` is computed from host arrival times.
    pub fn edge(&mut self, channel: Channel, t: f64, hint: i8) -> i64 {
        self.edge_with_period(channel, t, hint, 0.0)
    }

    /// [`edge`](Self::edge), but taking the `period` term from the firmware.
    ///
    /// `hw_period` is the platform's `dt_secs` for this pulse. In **rising-only
    /// mode** — which is the only mode this app subscribes in — `doovit_fw` 1.9.1
    /// defines that as the interval since the *same* edge on the same pin, i.e.
    /// exactly one full tooth cycle, and it is **measured by the PIO state
    /// machine, not by the CPU** (`dio.py:300-304`:
    /// `dt_secs = half_secs if edges == "both" else period_secs` … *"Both are
    /// PIO-measured (no CPU-tick jitter)"*). Pass `0.0` for "the firmware had
    /// none", which it reports on the first edge(s) of a pin and after a dropped
    /// transition.
    ///
    /// Why this matters: the discriminator is `2·gap` vs `period`. `gap` is
    /// cross-channel and has no hardware equivalent on the live pulse stream, so
    /// it must stay host-timed — but `period` does have one, and taking it from
    /// hardware fixes two things a host-computed period gets wrong:
    ///
    /// * **After a lost edge the host period is silently 3 quarter-cycles instead
    ///   of 4**, which inverts the comparison. The hardware reports `0.0` instead
    ///   of a plausible-but-wrong number, so the decoder can *know* it cannot
    ///   measure rather than confidently measuring backwards.
    /// * The threshold `AMBIGUITY_BAND · period` stops wobbling with host jitter,
    ///   so "unreadable" means the *waveform* was unreadable.
    ///
    /// It is **not** a complete fix for host jitter: a single late delivery
    /// inflates `gap`, and with a host period that error partially cancels
    /// (`2(g+δ) − (p+δ)`) whereas with a hardware period it does not
    /// (`2(g+δ) − p`). The hardware period is still the right choice — its
    /// failure mode is a *detected* miss rather than a silent inversion, and the
    /// two are cross-checked so jitter becomes visible in
    /// [`period_disagreements`](Self::period_disagreements) instead of quietly
    /// eroding the margin.
    pub fn edge_with_period(&mut self, channel: Channel, t: f64, hint: i8, hw_period: f64) -> i64 {
        let prev_same = self.last_rise[channel.idx()];
        self.last_rise[channel.idx()] = Some(t);

        let mut measured: i8 = 0;
        if hint == 1 || hint == -1 {
            measured = hint;
        } else if self.last_ch.is_none() {
            // First edge ever: nothing to measure against.
        } else if Some(channel) == self.last_ch {
            // Same channel twice: the other channel's rise was lost, or the
            // target reversed inside a quadrant. Cannot tell which.
            //
            // Every stored timestamp is now known-bad: the host "period" either
            // side of the discontinuity spans the wrong number of quarter cycles,
            // and measuring from it INVERTS the direction (a single lost edge
            // would otherwise flip the sign of everything that follows). Throw
            // the timing state away and re-measure from scratch; the next two
            // edges are held on the previous direction and the third is clean
            // again.
            self.missed += 1;
            self.last_rise = [None, None];
            self.last_rise[channel.idx()] = Some(t);
        } else {
            let gap = t - self.last_t;
            // Prefer the firmware's PIO-measured period; fall back to the host
            // interval only when the firmware had none to give.
            let period = if hw_period > 0.0 {
                self.hw_period_used += 1;
                if let Some(prev_same) = prev_same {
                    let host_period = t - prev_same;
                    if host_period > 0.0
                        && (host_period - hw_period).abs() > Self::PERIOD_DISAGREEMENT * hw_period
                    {
                        self.period_disagreements += 1;
                    }
                }
                hw_period
            } else {
                self.hw_period_missing += 1;
                // No hardware period. `prev_same` is the only fallback, and on
                // the first edge of a channel there isn't one either.
                match prev_same {
                    Some(prev_same) => t - prev_same,
                    None => 0.0,
                }
            };
            if gap > 0.0 && period > 0.0 {
                let skew = 2.0 * gap - period;
                if skew.abs() >= Self::AMBIGUITY_BAND * period {
                    // skew < 0 -> the gap just closed was the SHORT one, so the
                    // other channel led this one.
                    let other_led = skew < 0.0;
                    let b_leads = other_led == (channel == Channel::A);
                    measured = if b_leads { 1 } else { -1 };
                }
            }
        }

        self.last_ch = Some(channel);
        self.last_t = t;

        if measured != 0 {
            self.sense = measured;
        }
        if self.sense == 0 {
            // Nothing has established a direction yet. Guessing here would put
            // a systematic error into every run that starts by closing, so hold
            // the edge instead; it is flushed below within one cycle of motion.
            self.unsigned += 1;
            self.direction = 0;
            return 0;
        }
        if measured == 0 {
            self.ambiguous += 1;
        }

        self.direction = if self.invert { -self.sense } else { self.sense };
        // Flush any edges held before the first measurement, signed the same
        // way: they are part of the same motion that produced the measurement.
        let delta = self.direction as i64 * (1 + self.unsigned as i64);
        self.unsigned = 0;
        self.count += delta;
        delta
    }

    /// Consume one ordered poll batch of [`EdgeRecord`]s.
    ///
    /// This is the polled-ingest counterpart of [`edge`](Self::edge): same
    /// decode, fed from the platform's batched DI event log instead of live
    /// callbacks. Only rising records are decoded; anything else increments
    /// `filtered`.
    ///
    /// `hint_at(time_ms) -> +1/-1/0` is evaluated at each event's CAPTURE time,
    /// not decode time, because the platform syncs events 30-90 s late and a
    /// decode-time hint would mis-sign stragglers.
    ///
    /// Returns the applied count delta.
    pub fn feed_batch<F>(&mut self, edges: &[EdgeRecord], mut hint_at: F) -> i64
    where
        F: FnMut(i64) -> i8,
    {
        let mut delta = 0;
        for e in edges {
            if !e.level {
                self.filtered += 1;
                continue;
            }
            delta += self.edge(e.channel, e.time_ms as f64 / 1000.0, hint_at(e.time_ms));
        }
        delta
    }

    /// Reset the count to 0 (called on homing / manual zero).
    pub fn zero(&mut self) {
        self.count = 0;
    }

    /// Worst-case position error a single direction reversal can inject.
    ///
    /// This is a property of rising-edge-only sensing, not of this code, and no
    /// decoder can remove it. Channel A rises a quarter cycle *before* its
    /// falling edge when travelling one way and a quarter cycle *after* it when
    /// travelling the other, so the physical positions at which counts are
    /// emitted shift by half a cycle with direction. Depending on where in the
    /// cycle the reversal lands, an out-and-back move therefore finishes 0, 1 or
    /// 2 counts high — up to one whole tooth pitch — and the error persists
    /// (repeated reversals random-walk it further) until the next home.
    ///
    /// Capturing both edges would pin the count to the same four physical
    /// positions in either direction and remove this entirely.
    pub const fn reversal_backlash_counts() -> i64 {
        2
    }
}

// ---------------------------------------------------------------------------
// Rotational rate: revolutions + RPM helpers
// ---------------------------------------------------------------------------
// Like the decoder above these are pure and hardware-free, so the application
// can feed them samples and publish the results while they stay unit-testable.

/// Below this magnitude (rpm) the rotation reads as stopped, not cw/ccw.
pub const RPM_STOPPED_THRESHOLD: f64 = 0.5;

/// Decoded counts in one full revolution of the target wheel.
///
/// `pulses_per_rev` is the tooth count; decoding is 2x (the rising edge of each
/// of the two channels), so a full revolution is `2 * pulses_per_rev` counts.
pub fn counts_per_rev(pulses_per_rev: i64) -> i64 {
    2 * pulses_per_rev
}

/// Signed revolutions represented by `count` decoded edges. Returns 0.0 when
/// `counts_per_rev` is non-positive (misconfigured) rather than dividing by
/// zero.
pub fn revolutions(count: i64, counts_per_rev: i64) -> f64 {
    if counts_per_rev <= 0 {
        return 0.0;
    }
    count as f64 / counts_per_rev as f64
}

/// Classify gate travel from the change in decoded count alone.
///
/// `"opening"` / `"closing"` / `"stopped"`, decided purely by whether the pulse
/// count moved up, down, or not at all since the previous publish. This
/// deliberately does NOT go via height: `mm_per_count` is allowed to be 0 or
/// unset on a freshly-deployed install, which would make a speed-derived
/// direction read "stopped" while the gate was plainly moving.
pub fn travel_direction(count_delta: i64) -> &'static str {
    if count_delta > 0 {
        "opening"
    } else if count_delta < 0 {
        "closing"
    } else {
        "stopped"
    }
}

/// Classify a signed rpm into `"cw"` / `"ccw"` / `"stopped"`.
///
/// Positive rpm (increasing count) is "cw", negative is "ccw". Within
/// +/-`threshold` of zero it reads as stopped so sensor jitter can't flicker
/// the direction.
pub fn rotation_direction(rpm: f64, threshold: f64) -> &'static str {
    if rpm > threshold {
        "cw"
    } else if rpm < -threshold {
        "ccw"
    } else {
        "stopped"
    }
}

/// Sliding-window estimator of rotational speed from position samples.
///
/// Feed it one `(t, count)` sample per publish; it keeps only the samples
/// inside a trailing `window_s` window and estimates the rate from the oldest
/// surviving sample against the newest.
#[derive(Debug, Clone)]
pub struct RateEstimator {
    /// Width of the trailing window, in seconds. ~2 s suits a slow gate at the
    /// default 0.5 s publish period (a handful of samples per estimate).
    pub window_s: f64,
    samples: VecDeque<(f64, i64)>,
}

impl RateEstimator {
    pub fn new(window_s: f64) -> Self {
        Self { window_s, samples: VecDeque::new() }
    }

    pub fn len(&self) -> usize {
        self.samples.len()
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Append a sample and drop any that fell outside the trailing window.
    pub fn add(&mut self, t: f64, count: i64) {
        self.samples.push_back((t, count));
        while self.samples.len() > 1 && (t - self.samples[0].0) > self.window_s {
            self.samples.pop_front();
        }
    }

    /// Estimate signed revolutions-per-minute over the current window.
    ///
    /// Uses the oldest sample still within the window versus the newest.
    /// Returns 0.0 with fewer than two samples, a non-positive
    /// `counts_per_rev`, or a non-positive time span — all "can't tell yet".
    pub fn rpm(&self, counts_per_rev: i64) -> f64 {
        if self.samples.len() < 2 || counts_per_rev <= 0 {
            return 0.0;
        }
        let (t0, c0) = self.samples[0];
        let (t1, c1) = self.samples[self.samples.len() - 1];
        let dt = t1 - t0;
        if dt <= 0.0 {
            return 0.0;
        }
        (c1 - c0) as f64 / counts_per_rev as f64 / dt * 60.0
    }

    /// Forget all samples so a count discontinuity (e.g. a zero/home) can't
    /// read as a phantom rate spike.
    pub fn reset(&mut self) {
        self.samples.clear();
    }
}

impl Default for RateEstimator {
    fn default() -> Self {
        Self::new(2.0)
    }
}

/// A raw platform DI event, reduced to the three fields the merge needs.
///
/// `event` is the platform's event name. Real Doovit hardware emits
/// `"DI_R"`/`"DI_F"`; the docker platform-interface *simulator* emits
/// `"rising"`/`"falling"`/`"both"` instead, so both vocabularies are accepted
/// here rather than silently discarding every simulated event (which is what
/// the Python app does).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RawDiEvent {
    pub event_id: i64,
    pub event: String,
    pub time_ms: i64,
}

/// Classify a platform event name into a rising/falling level, or `None` for
/// anything that isn't a DI edge.
fn event_level(event: &str) -> Option<bool> {
    match event {
        "DI_R" | "rising" => Some(true),
        "DI_F" | "falling" => Some(false),
        _ => None,
    }
}

/// Merge per-channel DI event batches into one ordered edge list.
///
/// `batches` pairs a channel with that pin's raw event list as returned by
/// `fetch_di_events` — `event_id` is a sequence GLOBAL across pins, so sorting
/// by it recovers true cross-channel order. The platform returns duplicates and
/// re-serves old events on every poll, so entries are deduplicated by
/// `event_id` and only ids strictly greater than `last_id` survive.
///
/// Falling events are still passed through rather than dropped here, so
/// [`RisingEdgeDecoder::feed_batch`] can COUNT them (`filtered`) and make a pin
/// left on `irq_edge="both"` visible instead of silent.
///
/// Returns `(edges, newest_id)` where `edges` is sorted by `event_id` and
/// `newest_id` is the highest id seen across ALL input events (advance the
/// caller's cursor with it even when nothing decodable was new).
pub fn merge_new_edge_events(
    batches: &[(Channel, &[RawDiEvent])],
    last_id: i64,
) -> (Vec<EdgeRecord>, i64) {
    // BTreeMap gives the id ordering for free; `or_insert` keeps the first of
    // any duplicate ids (they are byte-identical repeats).
    let mut seen: BTreeMap<i64, EdgeRecord> = BTreeMap::new();
    let mut newest = last_id;
    for (channel, events) in batches {
        for e in *events {
            newest = newest.max(e.event_id);
            if e.event_id <= last_id {
                continue;
            }
            let Some(level) = event_level(&e.event) else {
                continue;
            };
            seen.entry(e.event_id).or_insert(EdgeRecord {
                event_id: e.event_id,
                channel: *channel,
                level,
                time_ms: e.time_ms,
            });
        }
    }
    (seen.into_values().collect(), newest)
}
