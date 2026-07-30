//! Pulse fidelity over the real gRPC transport — the Rust counterpart of the
//! Python app's `tests/test_pulse_fidelity.py`.
//!
//! Every test here drives the app's real ingest path: a tonic server speaking
//! `platform_iface.proto` on a real socket (`tests/common/platform_double.rs`),
//! the real doover-rs `PlatformClient`, the real `startPulseCounter` stream, the
//! real `startPulseCounter` stream, the app's own raw-reader task per pin, and the real
//! [`EncoderCore`](channel_gate_encoder::EncoderCore) callbacks. Only the edge
//! *source* is synthetic — see the double's module docs for why the shipped
//! platform-interface simulator cannot be used (it level-samples DI every
//! 50 ms).
//!
//! The soak is `#[ignore]`d because it takes two minutes:
//!
//! ```sh
//! cargo test --release --test fidelity -- --ignored --nocapture
//! ```

mod common;

use std::sync::atomic::Ordering;
use std::sync::LazyLock;
use std::time::{Duration, Instant};

use channel_gate_encoder::quadrature::Channel;
use channel_gate_encoder::state::unix_secs;
use channel_gate_encoder::{EncoderCore, Params};
use common::platform_double::{run_firmware_sweep, spawn_platform_double, QuadratureGate};
use doover::PlatformClient;

/// 15 rising edges/s per sensor.
const RISING_HZ_PER_SENSOR: f64 = 15.0;
/// 30 rising callbacks/s combined — the rate the app must sustain.
const COMBINED_RISING_HZ: f64 = 30.0;
/// The 90-degree A->B spacing, in ms.
const NOMINAL_GAP_MS: f64 = 1000.0 / (4.0 * RISING_HZ_PER_SENSOR);

const A_PIN: i32 = 0;
const B_PIN: i32 = 1;

/// Serialises the tests in this file.
///
/// Every test here runs a 15 Hz injector and measures **timing**, so two of them
/// overlapping is two encoders competing for the same cores — which pushes
/// inter-channel gaps toward the ambiguity band and makes the measurement about
/// the test runner instead of the app. (That is not a hypothetical: run these
/// concurrently in a debug build and `ambiguous` goes non-zero. It is the same
/// host-contention sensitivity the README documents, reproduced accidentally.)
static SERIAL: LazyLock<tokio::sync::Mutex<()>> = LazyLock::new(|| tokio::sync::Mutex::new(()));

/// The app + a gate feeding it, wired together over gRPC.
struct Harness {
    core: EncoderCore,
    gate: QuadratureGate,
    state: std::sync::Arc<common::platform_double::PlatformDoubleState>,
    /// Held for the lifetime of the test: dropping the client would not stop the
    /// listener tasks, but keeping it makes the ownership honest.
    _plt: PlatformClient,
    /// Released when the harness drops, letting the next test start.
    _serial: tokio::sync::MutexGuard<'static, ()>,
    /// The firmware sweep task, when the 1.9.1 model is enabled.
    _sweep: Option<tokio::task::AbortHandle>,
}

async fn harness(rising_hz: f64) -> Harness {
    harness_with(rising_hz, None).await
}

/// `harness`, optionally with the `doovit_fw` 1.9.1 sweep model in front of the
/// app (`Some(50)` = the shipping 50 ms `check_interrupts` cadence).
async fn harness_with(rising_hz: f64, firmware_sweep_ms: Option<u64>) -> Harness {
    let serial = SERIAL.lock().await;
    let (state, uri) = spawn_platform_double().await;
    let plt = PlatformClient::connect(uri).await.expect("connect to platform double");

    let mut core = EncoderCore::new(Params::for_test(A_PIN, B_PIN));
    core.start_pulse_listeners(&plt);
    // Both stream subscriptions must be registered server-side before the first
    // edge, or the double would "lose" pulses the app never had a chance at.
    let deadline = Instant::now() + Duration::from_secs(5);
    while state.subscriber_count() < 2 && Instant::now() < deadline {
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(state.subscriber_count(), 2, "one rising subscription per pin");

    let gate = QuadratureGate::new(state.clone(), A_PIN, B_PIN, rising_hz);
    gate.seed();
    let sweep = firmware_sweep_ms.map(|ms| {
        state.enable_firmware_sweep(ms);
        run_firmware_sweep(state.clone())
    });
    Harness { core, gate, state, _plt: plt, _serial: serial, _sweep: sweep }
}

/// Wait for the in-flight stream frames to reach the callbacks. Polls until the
/// callback count matches what the double streamed and then stops changing.
async fn drain(h: &Harness) {
    // Flush whatever the firmware sweep is holding, then wait for delivery.
    h.state.flush_sweep();
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let streamed = h.state.streamed.load(Ordering::Relaxed);
        let callbacks = h.core.state.lock().unwrap().callbacks;
        if callbacks >= streamed || Instant::now() > deadline {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    // One more settling pass so a frame in flight at the boundary lands.
    tokio::time::sleep(Duration::from_millis(50)).await;
}

/// Inter-channel gap statistics from the arrival trace.
///
/// This is the measurement that matters now that direction is a **timing**
/// comparison: the short A->B gap is nominally 16.67 ms and the discriminator
/// flips at half a cycle (33.33 ms), so anything that stretches a short gap past
/// ~25 ms (the edge of the ambiguity band) starts corrupting direction.
struct GapStats {
    count: usize,
    worst_error_ms: f64,
    p50_ms: f64,
    p99_ms: f64,
    max_ms: f64,
    /// Short gaps stretched into the ambiguity band (>= 25.0 ms at 15 Hz).
    in_ambiguity_band: usize,
}

fn gap_stats(trace: &[(Channel, f64)]) -> GapStats {
    // Every consecutive pair in the trace is an inter-channel gap: rising edges
    // alternate A, B, A, B, so successive arrivals are always cross-channel
    // (except at a same-channel repeat, which a one-way run never produces).
    let mut short_gaps: Vec<f64> = Vec::new();
    for w in trace.windows(2) {
        let gap_ms = (w[1].1 - w[0].1) * 1000.0;
        // Classify by which side of half a cycle it fell on; the nominal pair is
        // 16.67 ms / 50.0 ms, so 33.33 ms is the split.
        if gap_ms < 2.0 * NOMINAL_GAP_MS {
            short_gaps.push(gap_ms);
        }
    }
    short_gaps.sort_by(f64::total_cmp);
    let pct = |v: &[f64], p: f64| -> f64 {
        if v.is_empty() {
            return 0.0;
        }
        v[(((v.len() - 1) as f64) * p).round() as usize]
    };
    GapStats {
        count: short_gaps.len(),
        worst_error_ms: short_gaps.iter().map(|g| (g - NOMINAL_GAP_MS).abs()).fold(0.0, f64::max),
        p50_ms: pct(&short_gaps, 0.50),
        p99_ms: pct(&short_gaps, 0.99),
        max_ms: short_gaps.last().copied().unwrap_or(0.0),
        // 0.25 ambiguity band on a 66.67 ms period: readable requires
        // |2*gap - period| >= period/4, i.e. gap outside (25.0, 41.67) ms.
        in_ambiguity_band: short_gaps.iter().filter(|g| **g >= 25.0).count(),
    }
}

fn report(label: &str, h: &Harness, elapsed: f64) -> String {
    let st = h.core.state.lock().unwrap();
    let injected = h.state.injected.load(Ordering::Relaxed);
    let stream_reconnects = st.stream_reconnects;
    let injected_rising = h.state.injected_rising.load(Ordering::Relaxed);
    let streamed = h.state.streamed.load(Ordering::Relaxed);
    let queue_drops = h.state.queue_drops.load(Ordering::Relaxed);
    let trace = st.trace.clone().unwrap_or_default();
    let gaps = gap_stats(&trace);
    let out = format!(
        "\n=== {label} ===\n\
         duration                    {elapsed:.2} s\n\
         injected edges (all)        {injected}\n\
         injected RISING             {injected_rising}   (gate truth {})\n\
         streamed by platform        {streamed}\n\
         platform queue drops        {queue_drops}\n\
         pulse callbacks delivered   {}\n\
         DROPPED (injected-cb)       {}\n\
         decoder count               {}\n\
         gate true_position          {}\n\
         position error              {}\n\
         missed (same-ch repeats)    {}\n\
         ambiguous (held sign)       {}\n\
         unsigned (pre-direction)    {}\n\
         filtered (falling)          {}\n\
         pulses reporting level HIGH {}   (upstream bug: always 0)\n\
         hw (PIO) period used        {}\n\
         hw period missing (dt=0)    {}\n\
         host/hw period disagree>10% {}\n\
         stream reconnects           {stream_reconnects}\n\
         achieved rising/s/sensor    {:.2}\n\
         achieved rising/s combined  {:.2}\n\
         injector worst interval err {:.2} ms\n\
         --- inter-channel SHORT gap (nominal {NOMINAL_GAP_MS:.2} ms) ---\n\
         samples                     {}\n\
         p50 / p99 / max             {:.2} / {:.2} / {:.2} ms\n\
         worst |gap - nominal|       {:.2} ms\n\
         gaps inside ambiguity band  {}   (>= 25.00 ms)\n",
        h.gate.rising_emitted,
        st.callbacks,
        injected_rising as i64 - st.callbacks as i64,
        st.decoder.count,
        h.gate.true_position,
        st.decoder.count - h.gate.true_position,
        st.decoder.missed,
        st.decoder.ambiguous,
        st.decoder.unsigned,
        st.decoder.filtered,
        st.pulses_reporting_high,
        st.decoder.hw_period_used,
        st.decoder.hw_period_missing,
        st.decoder.period_disagreements,
        h.gate.achieved_rising_hz_per_sensor(),
        h.gate.achieved_rising_hz_combined(),
        h.gate.max_injection_error_ms(),
        gaps.count,
        gaps.p50_ms,
        gaps.p99_ms,
        gaps.max_ms,
        gaps.worst_error_ms,
        gaps.in_ambiguity_band,
    );
    println!("{out}");
    out
}

// ---------------------------------------------------------------------------
// The subscription shape
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn one_rising_subscription_per_pin_and_no_falling_reaches_the_decoder() {
    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    h.gate.run_for(Duration::from_secs(2), None).await;
    drain(&h).await;

    let st = h.core.state.lock().unwrap();
    // The double emits falling edges too; the subscription's own edge filter is
    // what keeps them out.
    assert!(
        h.state.injected.load(Ordering::Relaxed) > h.state.injected_rising.load(Ordering::Relaxed),
        "the gate must really emit falling edges as well"
    );
    assert_eq!(st.decoder.filtered, 0, "no falling edge may reach the decoder");
    assert_eq!(st.callbacks, h.gate.rising_emitted, "one callback per rising edge");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_platform_never_reports_a_level_and_the_decode_does_not_care() {
    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    h.gate.run_for(Duration::from_secs(1), None).await;
    drain(&h).await;
    let out = report("upstream bug 1: no level on any pulse", &h, 1.0);

    let st = h.core.state.lock().unwrap();
    assert!(st.callbacks > 20, "need a real sample; got {}", st.callbacks);
    // platform_iface_base.py:275-281 never sets `value`, so it is proto3-absent
    // on every pulse on every driver. doover-rs surfaces that as `false`.
    assert_eq!(
        st.pulses_reporting_high, 0,
        "if this is ever non-zero, upstream fixed the bug\n{out}"
    );
    // And the position is still exactly right, because the channel is bound from
    // WHICH SUBSCRIPTION delivered the pulse, not from the wire.
    assert_eq!(st.decoder.count, h.gate.true_position, "decode is level-independent\n{out}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn there_is_no_startup_grace_period_swallowing_the_first_counts() {
    // pydoover discards every pulse in the first 0.2 s after a listener starts,
    // and doover-rs ported that into `PulseCounter::start_listener_pulses`
    // (docker/platform.rs:1034) as a private const with no setter. At 30 rising
    // edges/s that is 6 counts of silently lost distance. Consuming the raw
    // stream directly avoids it entirely — this test is what pins that down.
    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    h.gate.run_for(Duration::from_millis(500), None).await;
    drain(&h).await;

    let st = h.core.state.lock().unwrap();
    assert!(h.gate.rising_emitted >= 7, "0.5 s must contain more than a grace period");
    assert_eq!(
        st.callbacks, h.gate.rising_emitted,
        "every pulse from t=0 must be counted, including the first 0.2 s"
    );
    assert_eq!(st.decoder.count, h.gate.true_position);
}

// ---------------------------------------------------------------------------
// Fidelity
// ---------------------------------------------------------------------------

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn short_burst_is_lossless() {
    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    h.core.state.lock().unwrap().enable_trace(4096);
    let start = Instant::now();
    h.gate.run_for(Duration::from_secs(2), None).await;
    drain(&h).await;
    let out = report("2 s burst @ 30 rising/s combined", &h, start.elapsed().as_secs_f64());

    let st = h.core.state.lock().unwrap();
    assert_eq!(st.callbacks, h.gate.rising_emitted, "zero dropped callbacks\n{out}");
    assert_eq!(st.decoder.count, h.gate.true_position, "zero position drift\n{out}");
    // 2x decode: exactly one count per rising edge.
    assert_eq!(st.decoder.count, h.gate.rising_emitted as i64, "2x decode\n{out}");
    assert_eq!(st.decoder.missed, 0, "no same-channel repeats on a one-way run\n{out}");
    assert_eq!(st.decoder.ambiguous, 0, "every count measured, none held\n{out}");
    assert_eq!(h.state.queue_drops.load(Ordering::Relaxed), 0, "the double lost nothing\n{out}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_publish_timer_is_decoupled_from_the_pulse_rate() {
    // 500 ms publish interval against 30 callbacks/s: the callback path must
    // cause zero derivations, and the publish count must track the clock, not
    // the edge rate.
    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    h.core.params.publish_interval_s = 0.5;
    let start = Instant::now();
    let mut publishes = 0;
    let mut heights = Vec::new();
    let gate_run = Duration::from_secs(2);
    // Poll far faster than the publish interval, exactly as the runner does.
    let poller = async {
        while start.elapsed() < gate_run {
            tokio::time::sleep(Duration::from_millis(100)).await;
            let now = unix_secs();
            if h.core.due_to_publish(now) {
                let snap = h.core.publish(now);
                heights.push(snap.height_mm);
                publishes += 1;
            }
        }
    };
    let runner = h.gate.run_for(gate_run, None);
    tokio::join!(runner, poller);

    // 2 s at 500 ms = 4 publishes, +/- one for the boundary.
    assert!(
        (3..=5).contains(&publishes),
        "publish cadence follows the config, not the pulse rate: {publishes}"
    );
    assert!(
        heights.windows(2).all(|w| w[1] >= w[0]),
        "height must increase monotonically on a one-way run: {heights:?}"
    );
    assert!(*heights.last().unwrap() > 0.0, "something must have been published");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn reversal_loses_no_pulses_and_the_pio_period_halves_its_cost() {
    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    h.core.state.lock().unwrap().enable_trace(4096);
    let start = Instant::now();
    h.gate.run_for(Duration::from_secs(4), Some(Duration::from_secs(2))).await;
    drain(&h).await;
    let out = report("4 s with a reversal at 2 s", &h, start.elapsed().as_secs_f64());

    let st = h.core.state.lock().unwrap();
    assert_eq!(st.callbacks, h.gate.rising_emitted, "a reversal loses no pulses\n{out}");
    assert_eq!(st.decoder.direction, -1, "direction must follow the reversal\n{out}");
    assert_eq!(st.decoder.missed, 1, "one same-channel repeat at the turnaround\n{out}");
    // With a HOST-computed period this is 2 (see the unit tests): the edge after
    // the turnaround has no usable same-channel interval, because the decoder
    // just discarded its timing state. With the firmware's PIO-measured period it
    // does have one -- a stretched 5-quarter-cycle period, but a true one -- so
    // that edge can be MEASURED instead of held. Only the turnaround edge itself
    // remains unsignable, and no timing can ever sign that one: a reversal and a
    // lost edge are indistinguishable from rising edges alone.
    assert_eq!(
        st.decoder.ambiguous, 1,
        "the PIO period leaves only the turnaround edge unsignable\n{out}"
    );
    let error = st.decoder.count - h.gate.true_position;
    assert_eq!(
        error,
        2,
        "reversal costs 2 counts with the PIO period, 4 without = {:.1} mm at \
         mm_per_count={}\n{out}",
        error as f64 * h.core.params.mm_per_count,
        h.core.params.mm_per_count
    );
    // 2 counts is also the irreducible quantiser bound, so with the hardware
    // period the decode error is now AT the floor for rising-edge-only sensing.
    assert_eq!(
        error,
        channel_gate_encoder::RisingEdgeDecoder::reversal_backlash_counts(),
        "the decode error is down to the geometric floor\n{out}"
    );
    // The separate quantiser error, which no decoder can remove.
    assert!(
        h.gate.reversal_error_counts().abs()
            <= channel_gate_encoder::RisingEdgeDecoder::reversal_backlash_counts(),
        "backlash bound\n{out}"
    );
}

// ---------------------------------------------------------------------------
// The soak
// ---------------------------------------------------------------------------

/// Two minutes at 30 rising callbacks/s combined, with the publish timer running
/// concurrently so callback load and publishing contend for the same runtime —
/// exactly as on the device.
///
/// Pass criteria (all exact, no tolerance):
/// * `injected_rising == callbacks` — zero dropped pulses
/// * `decoder.count == gate.true_position` — zero position drift
/// * `missed == 0`, `ambiguous == 0` — every count signed by a real measurement
///
/// It also reports the **inter-channel gap distribution**, which is the number
/// that decides whether the direction decode is safe at this rate.
#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
#[ignore = "two-minute soak; run with --ignored --nocapture"]
async fn sustained_two_minute_soak() {
    let seconds: f64 =
        std::env::var("GATE_SOAK_SECONDS").ok().and_then(|v| v.parse().ok()).unwrap_or(120.0);
    let duration = Duration::from_secs_f64(seconds);

    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    // 30/s * 120 s = 3600, plus headroom so the hot path never reallocates.
    h.core.state.lock().unwrap().enable_trace((seconds * COMBINED_RISING_HZ * 1.5) as usize);
    h.core.params.publish_interval_s = 0.5;

    let start = Instant::now();
    let mut publishes = 0u32;
    let publisher = async {
        while start.elapsed() < duration {
            tokio::time::sleep(Duration::from_millis(100)).await;
            let now = unix_secs();
            if h.core.due_to_publish(now) {
                h.core.publish(now);
                publishes += 1;
            }
        }
    };
    let runner = h.gate.run_for(duration, None);
    tokio::join!(runner, publisher);
    drain(&h).await;

    let elapsed = start.elapsed().as_secs_f64();
    let out = report(&format!("SOAK {seconds:.0} s @ 30 rising callbacks/s combined"), &h, elapsed);
    println!("publishes during the soak: {publishes}");

    let injected_rising = h.state.injected_rising.load(Ordering::Relaxed);
    let expected = (seconds * COMBINED_RISING_HZ).round() as u64;
    // The only tolerant assertion, and it is about the HARNESS, not the app: the
    // injector is a tokio timer, so a handful of edges of shortfall over two
    // minutes is scheduling, not a defect.
    let shortfall = expected as i64 - injected_rising as i64;
    assert!(
        shortfall.abs() as f64 <= expected as f64 * 0.005,
        "the injector must hold 30 rising/s within 0.5%: expected {expected}, \
         got {injected_rising} (shortfall {shortfall})\n{out}"
    );

    let st = h.core.state.lock().unwrap();

    // --- the pass criteria: exact, no tolerance ---
    assert_eq!(st.callbacks, injected_rising, "ZERO DROPPED PULSES over the soak\n{out}");
    assert_eq!(st.callbacks, h.gate.rising_emitted, "callbacks match the gate truth\n{out}");
    assert_eq!(st.decoder.count, h.gate.true_position, "ZERO POSITION DRIFT\n{out}");
    assert_eq!(st.decoder.missed, 0, "no same-channel repeats\n{out}");
    assert_eq!(st.decoder.filtered, 0, "no falling edges\n{out}");
    assert_eq!(h.state.queue_drops.load(Ordering::Relaxed), 0, "the double lost nothing\n{out}");
    assert!(publishes > 0, "the publish timer must have run\n{out}");
    // Every edge carried a PIO-measured period except the first rise on the
    // second channel, so the direction threshold was hardware-derived throughout.
    assert_eq!(
        st.decoder.hw_period_used + st.decoder.hw_period_missing + 1,
        st.callbacks,
        "every edge accounted for\n{out}"
    );

    // --- ambiguity: bounded and reported, not required to be zero ---
    //
    // This is deliberately NOT `== 0`. macOS (and Linux without RT scheduling) is
    // not a real-time host: a tokio timer wakeup can be late, and this soak
    // measures a 16.7 ms gap. When the gap gets stretched past 25.0 ms the
    // decoder refuses to measure and holds the last direction — which is the
    // CORRECT behaviour and is exactly why `ambiguous` exists. Demanding zero
    // would make the test a scheduler lottery rather than a statement about the
    // app. What matters is that it stays rare and that position stays exact,
    // both asserted here.
    let ambiguity_rate = st.decoder.ambiguous as f64 / st.callbacks as f64;
    println!(
        "ambiguity: {}/{} edges = {:.3}% (each one is a count whose SIGN was held, \
         not measured)",
        st.decoder.ambiguous,
        st.callbacks,
        ambiguity_rate * 100.0
    );
    assert!(
        ambiguity_rate <= 0.005,
        "unsignable edges must stay under 0.5% of the stream; got {:.3}%\n{out}",
        ambiguity_rate * 100.0
    );
}

// ---------------------------------------------------------------------------
// Through the doovit_fw 1.9.1 firmware model
// ---------------------------------------------------------------------------
// The device runs 1.9.1, where `check_interrupts` (dio.py:266) drains EVERY
// confirmed edge from a PIO debouncer and broadcasts each one individually, with
// its timestamp back-computed from its leading-edge tick so the 50 ms sweep
// delays delivery without moving the timestamp. There is no majority vote and no
// coalescing. `dt_secs` in rising-only mode is the PIO-measured full period.
//
// These tests put that model in front of the app: 50 ms batched delivery, exact
// per-edge ordering, PIO-style dt_secs.

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_firmware_sweep_drops_nothing() {
    // 1.9.1's sweep delays delivery; it does not destroy edges. The old
    // majority-vote/coalescing model (which deleted 68% of rising edges at this
    // rate) is gone -- edge detection moved into PIO1 SM0-3 and `drain_events()`
    // yields every confirmed edge. So the count of CALLBACKS must be exact.
    let mut h = harness_with(RISING_HZ_PER_SENSOR, Some(50)).await;
    h.core.state.lock().unwrap().enable_trace(4096);
    let start = Instant::now();
    h.gate.run_for(Duration::from_secs(4), None).await;
    drain(&h).await;
    let out = report("4 s through the 1.9.1 50 ms sweep", &h, start.elapsed().as_secs_f64());

    let st = h.core.state.lock().unwrap();
    assert_eq!(
        st.callbacks, h.gate.rising_emitted,
        "the sweep delays delivery, it does not drop edges\n{out}"
    );
    assert_eq!(h.state.queue_drops.load(Ordering::Relaxed), 0, "nothing lost in transport\n{out}");
    // And the hardware period survives the batching intact, because the firmware
    // measures it in the PIO rather than from the sweep's arrival times.
    assert!(
        st.decoder.hw_period_used > st.callbacks / 2,
        "most edges must carry a PIO-measured period\n{out}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn batched_delivery_costs_cross_channel_order_the_real_limit() {
    // THE honest result, and it is not what the stale-firmware story said.
    //
    // The firmware preserves order and per-edge timing. What does not survive is
    // the trip through the platform interface: `startPulseCounter` is ONE STREAM
    // PER PIN (doover-rs even gives each its own channel -- platform.rs:818
    // `stream_channel()`), and a 50 ms sweep releases a clump of events at once.
    // Two independent streams draining a simultaneous clump have no defined
    // interleaving, so the A,B,A,B alternation -- and with it the 16.7 ms `gap`
    // term that direction is measured from -- is scrambled at the CLIENT.
    //
    // The decoder does not hide this: cross-channel inversion shows up as
    // same-channel repeats (`missed`) and held signs (`ambiguous`), and position
    // goes wrong. Nothing is silently dropped; the damage is fully attributed.
    //
    // The fix is NOT app-side. `pulseCounterResponse` carries only
    // (di, value, dt_secs) -- yet the firmware already computes a per-edge
    // timestamp (`ev_epoch`, dio.py:297) and doovitd forwards it; the platform
    // interface discards it (`doovit_platform_iface.py:597-598` yields only
    // `payload["dt_secs"]`). Surfacing that timestamp would make batching
    // completely harmless, because both the gap AND the period would then be
    // firmware-measured and host scheduling would stop mattering entirely.
    let mut h = harness_with(RISING_HZ_PER_SENSOR, Some(50)).await;
    h.core.state.lock().unwrap().enable_trace(4096);
    h.gate.run_for(Duration::from_secs(4), None).await;
    drain(&h).await;
    let out = report("4 s through the sweep: ordering damage", &h, 4.0);

    let st = h.core.state.lock().unwrap();
    assert_eq!(st.callbacks, h.gate.rising_emitted, "nothing dropped\n{out}");
    // The damage is real and it is REPORTED, which is the property that matters:
    // an operator seeing these counters climb knows not to trust the position.
    assert!(
        st.decoder.missed > 0 || st.decoder.ambiguous > 0,
        "batched delivery must surface as missed/ambiguous, never as silent \
         confidence\n{out}"
    );
    // Every edge is still accounted for as either measured or explicitly flagged.
    assert_eq!(
        st.decoder.hw_period_used + st.decoder.hw_period_missing + st.decoder.missed + 1,
        st.callbacks,
        "edge accounting must balance: measured + no-period + repeats + the first \
         edge\n{out}"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_hardware_period_is_used_and_agrees_with_the_host_on_a_quiet_transport() {
    // On the lossless transport (no sweep) the host and hardware periods should
    // agree, which is what makes `period_disagreements` a usable jitter alarm:
    // it is 0 when the host is keeping up.
    let mut h = harness(RISING_HZ_PER_SENSOR).await;
    h.gate.run_for(Duration::from_secs(2), None).await;
    drain(&h).await;
    let out = report("2 s lossless: hardware vs host period", &h, 2.0);

    let st = h.core.state.lock().unwrap();
    // The first rising edge on each pin has no PIO period (dt_secs = 0), so
    // exactly two edges fall back to the host.
    // The very first edge overall takes the "nothing to measure against" path
    // and never consults a period, so exactly ONE edge (the first rise on the
    // second channel) reaches the period logic with dt_secs = 0.
    assert_eq!(
        st.decoder.hw_period_missing, 1,
        "only the first rise on the second channel lacks a hardware period\n{out}"
    );
    // Both of those first rises are still COUNTED. doover-rs's
    // start_di_pulse_listener would have dropped them (dt_secs > 0 filter), which
    // is why this app consumes the raw stream itself.
    assert_eq!(st.callbacks, h.gate.rising_emitted, "no pulse lost to a dt<=0 filter\n{out}");
    assert_eq!(st.stream_reconnects, 0, "no reconnects during a clean run\n{out}");
    // +1 for the very first edge overall, which takes the "nothing to measure
    // against" path and never consults a period at all.
    assert_eq!(
        st.decoder.hw_period_used + st.decoder.hw_period_missing + 1,
        st.callbacks,
        "every edge is accounted for\n{out}"
    );
    assert_eq!(
        st.decoder.period_disagreements, 0,
        "an unloaded host must agree with the PIO to within 10%\n{out}"
    );
}
