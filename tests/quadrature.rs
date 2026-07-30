//! Unit tests for the pure decode logic — the Rust counterpart of the Python
//! app's `tests/test_quadrature.py`.
//!
//! Everything here is at the **specified rate**: 15 rising edges/s per sensor,
//! so a full tooth cycle is 66.67 ms, the A-rise -> B-rise gap is 16.67 ms (90
//! deg) and the B-rise -> A-rise gap is 50.0 ms. Those three numbers appear
//! literally in the tests rather than as symbols, because the whole design
//! hinges on the decoder being able to tell 16.7 ms from 50.0 ms.

use channel_gate_encoder::quadrature::{
    counts_per_rev, merge_new_edge_events, revolutions, rotation_direction, travel_direction,
    Channel, EdgeRecord, RateEstimator, RawDiEvent, RisingEdgeDecoder,
};

/// 15 rising edges/s per sensor.
const RISING_HZ: f64 = 15.0;
/// Full tooth cycle at 15 Hz: 66.667 ms.
const CYCLE: f64 = 1.0 / RISING_HZ;
/// The 90-degree A->B spacing: 16.667 ms.
const QUARTER: f64 = CYCLE / 4.0;
/// The long gap, three quarters of a cycle: 50.0 ms.
const THREE_QUARTER: f64 = 3.0 * CYCLE / 4.0;

/// Drive `cycles` full tooth cycles in the forward sense (B's rise leads A's),
/// starting at `t0`, and return the time after the last edge.
///
/// Forward means: B rises, then a quarter cycle later A rises, then three
/// quarters later B rises again. That is the `00 -> 01 -> 11 -> 10` Gray walk
/// the platform simulators emit.
fn drive_forward(dec: &mut RisingEdgeDecoder, cycles: usize, mut t: f64) -> f64 {
    for _ in 0..cycles {
        dec.edge(Channel::B, t, 0);
        t += QUARTER;
        dec.edge(Channel::A, t, 0);
        t += THREE_QUARTER;
    }
    t
}

/// The reverse sense: A rises, then a quarter cycle later B rises.
fn drive_reverse(dec: &mut RisingEdgeDecoder, cycles: usize, mut t: f64) -> f64 {
    for _ in 0..cycles {
        dec.edge(Channel::A, t, 0);
        t += QUARTER;
        dec.edge(Channel::B, t, 0);
        t += THREE_QUARTER;
    }
    t
}

/// Reverse the Gray walk immediately after a forward A-rise, then continue in
/// the reverse sense for `cycles` more tooth cycles. Returns the time after the
/// last edge.
///
/// The exact walk, which is where the whole reversal cost comes from. Forward,
/// A rises entering phase 2 (`11`). Reversing from there goes `11 -> 01` (A
/// falls), `01 -> 00` (B falls), `00 -> 10` (**A rises**) — so the very next
/// rising edge is A again, three quarter cycles later: a **same-channel
/// repeat** carrying no direction information. After that the reverse steady
/// state is B, then A three quarters later (the mirror of `drive_forward`).
fn reverse_after_a_rise(dec: &mut RisingEdgeDecoder, cycles: usize, mut t: f64) -> f64 {
    // The turnaround: A rises again, 3 quarter cycles after the forward A rise.
    dec.edge(Channel::A, t, 0);
    t += QUARTER;
    for _ in 0..cycles {
        dec.edge(Channel::B, t, 0);
        t += THREE_QUARTER;
        dec.edge(Channel::A, t, 0);
        t += QUARTER;
    }
    t
}

// ---------------------------------------------------------------------------
// The 90-degree spacing itself
// ---------------------------------------------------------------------------

#[test]
fn the_three_headline_timings() {
    assert!((CYCLE * 1000.0 - 66.667).abs() < 0.01, "cycle = {} ms", CYCLE * 1000.0);
    assert!((QUARTER * 1000.0 - 16.667).abs() < 0.01, "A->B gap = {} ms", QUARTER * 1000.0);
    assert!(
        (THREE_QUARTER * 1000.0 - 50.0).abs() < 0.01,
        "B->A gap = {} ms",
        THREE_QUARTER * 1000.0
    );
}

#[test]
fn forward_run_counts_two_per_cycle_and_signs_them_positive() {
    let mut dec = RisingEdgeDecoder::new(false);
    drive_forward(&mut dec, 10, 0.0);

    // 2x decode: 10 tooth cycles = 20 rising edges = 20 counts.
    assert_eq!(dec.count, 20, "2x decoding: one count per rising edge");
    assert_eq!(dec.sense, 1, "B leads A is the positive sense");
    assert_eq!(dec.direction, 1);
    assert_eq!(dec.missed, 0);
    assert_eq!(dec.unsigned, 0, "held edges must be flushed once direction lands");
    // The first two edges have no same-channel period to measure against, so
    // they are held and then flushed by the third — they are NOT ambiguous.
    assert_eq!(dec.ambiguous, 0, "every count after the first cycle is measured");
}

#[test]
fn reverse_run_counts_negative() {
    let mut dec = RisingEdgeDecoder::new(false);
    drive_reverse(&mut dec, 10, 0.0);

    assert_eq!(dec.count, -20);
    assert_eq!(dec.sense, -1, "A leads B is the negative sense");
    assert_eq!(dec.direction, -1);
    assert_eq!(dec.missed, 0);
    assert_eq!(dec.ambiguous, 0);
}

#[test]
fn invert_flips_the_sign_without_touching_the_sense() {
    let mut dec = RisingEdgeDecoder::new(true);
    drive_forward(&mut dec, 10, 0.0);

    assert_eq!(dec.count, -20, "invert negates every step");
    assert_eq!(dec.sense, 1, "the raw sensor sense is unchanged");
    assert_eq!(dec.direction, -1, "direction is the applied sign");
}

#[test]
fn the_first_two_edges_are_held_not_guessed() {
    let mut dec = RisingEdgeDecoder::new(false);
    // Edge 1: nothing to measure against.
    assert_eq!(dec.edge(Channel::B, 0.0, 0), 0);
    assert_eq!((dec.count, dec.unsigned), (0, 1));
    // Edge 2: different channel, but B has no previous rise to give a period.
    assert_eq!(dec.edge(Channel::A, QUARTER, 0), 0);
    assert_eq!((dec.count, dec.unsigned), (0, 2));
    // Edge 3: B now has a full-cycle period, so direction is measurable and the
    // whole held run is flushed at once — 3 counts in one step.
    assert_eq!(dec.edge(Channel::B, CYCLE, 0), 3);
    assert_eq!((dec.count, dec.unsigned), (3, 0));
    assert_eq!(dec.ambiguous, 0, "a flushed hold is not an ambiguous count");
}

// ---------------------------------------------------------------------------
// Reversal
// ---------------------------------------------------------------------------

#[test]
fn reversal_with_a_host_computed_period_costs_four_counts() {
    let mut dec = RisingEdgeDecoder::new(false);
    // 10 cycles forward = 20 rising edges, all measured.
    let t = drive_forward(&mut dec, 10, 0.0);
    assert_eq!(dec.count, 20);

    // Reverse. 1 turnaround edge + 10 cycles x 2 = 21 rising edges backwards,
    // so the true 2x position is 20 - 21 = -1.
    let t = reverse_after_a_rise(&mut dec, 10, t);
    let truth = 20i64 - 21;

    assert_eq!(dec.direction, -1, "direction must follow the reversal");
    assert_eq!(dec.missed, 1, "the same-channel repeat at the turnaround");
    // Exactly two edges around the turnaround carry no readable timing — the
    // repeat itself, and the edge after it whose timing state was just
    // discarded — so both are signed with the OLD direction. 2 edges, each 2
    // counts wrong (+1 instead of -1) = 4 counts of permanent error.
    assert_eq!(dec.ambiguous, 2, "exactly two edges could not be signed by timing");
    assert_eq!(dec.count - truth, 4, "the reversal cost is exactly 4 counts");

    // And it does not heal: the count integrates, so the offset is still there
    // 10 cycles later. Only a home clears it.
    let after = dec.count;
    let mut t2 = t;
    for _ in 0..10 {
        dec.edge(Channel::B, t2, 0);
        t2 += THREE_QUARTER;
        dec.edge(Channel::A, t2, 0);
        t2 += QUARTER;
    }
    assert_eq!(dec.count, after - 20, "the 4-count offset persists, unchanged");
    assert_eq!(dec.ambiguous, 2, "no new ambiguity once the timing settles");
}

/// The same reversal, but with the firmware's PIO-measured period supplied for
/// every edge — which is what the app actually gets on `doovit_fw` 1.9.1.
#[test]
fn reversal_with_the_pio_period_costs_only_two_counts() {
    let mut dec = RisingEdgeDecoder::new(false);
    // Forward, 10 cycles. The hardware period for any steady-state edge is one
    // full tooth cycle; the first rise on each channel has none (0.0).
    let mut t = 0.0;
    let mut seen_a = false;
    let mut seen_b = false;
    for _ in 0..10 {
        dec.edge_with_period(Channel::B, t, 0, if seen_b { CYCLE } else { 0.0 });
        seen_b = true;
        t += QUARTER;
        dec.edge_with_period(Channel::A, t, 0, if seen_a { CYCLE } else { 0.0 });
        seen_a = true;
        t += THREE_QUARTER;
    }
    assert_eq!(dec.count, 20);

    // The turnaround: A rises again 3 quarter cycles later, so the PIO period for
    // that A edge is 3 quarters of a cycle.
    dec.edge_with_period(Channel::A, t, 0, THREE_QUARTER);
    t += QUARTER;
    // B's next rise is 5 quarter cycles after its previous one (the turnaround
    // stretched it) -- a real measured interval, so the decoder CAN sign this
    // edge, where a host-computed period left it with nothing.
    dec.edge_with_period(Channel::B, t, 0, 5.0 * QUARTER);
    t += THREE_QUARTER;
    dec.edge_with_period(Channel::A, t, 0, CYCLE);
    t += QUARTER;
    for _ in 0..9 {
        dec.edge_with_period(Channel::B, t, 0, CYCLE);
        t += THREE_QUARTER;
        dec.edge_with_period(Channel::A, t, 0, CYCLE);
        t += QUARTER;
    }

    let truth = 20i64 - 21;
    assert_eq!(dec.direction, -1, "direction still follows the reversal");
    assert_eq!(dec.missed, 1, "still one same-channel repeat");
    assert_eq!(dec.ambiguous, 1, "only the turnaround edge is unsignable now");
    assert_eq!(dec.count - truth, 2, "2 counts, half the host-period cost");
    // Which is exactly the irreducible geometric bound.
    assert_eq!(dec.count - truth, RisingEdgeDecoder::reversal_backlash_counts());
}

/// `dt_secs == 0.0` means "the firmware had no period", not "zero seconds".
#[test]
fn a_zero_hardware_period_falls_back_and_is_counted() {
    let mut dec = RisingEdgeDecoder::new(false);
    // Establish direction with real periods.
    let mut t = 0.0;
    dec.edge_with_period(Channel::B, t, 0, 0.0);
    t += QUARTER;
    dec.edge_with_period(Channel::A, t, 0, 0.0);
    t += THREE_QUARTER;
    dec.edge_with_period(Channel::B, t, 0, CYCLE);
    assert_eq!(dec.count, 3, "the held run is flushed");
    assert_eq!(dec.hw_period_used, 1);
    // Two edges lacked a hardware period; the first edge overall never consults
    // one, so only the second counts as "missing".
    assert_eq!(dec.hw_period_missing, 1);

    // Now a pulse the firmware could not period-stamp (post dropped-transition).
    t += QUARTER;
    dec.edge_with_period(Channel::A, t, 0, 0.0);
    assert_eq!(dec.hw_period_missing, 2, "counted as a missing hardware period");
    assert_eq!(dec.count, 4, "but the edge itself is still COUNTED, never dropped");
}

/// Host/hardware period disagreement is the app's jitter alarm.
#[test]
fn a_late_delivery_shows_up_as_a_period_disagreement() {
    let mut dec = RisingEdgeDecoder::new(false);
    let mut t = 0.0;
    dec.edge_with_period(Channel::B, t, 0, 0.0);
    t += QUARTER;
    dec.edge_with_period(Channel::A, t, 0, 0.0);
    t += THREE_QUARTER;
    dec.edge_with_period(Channel::B, t, 0, CYCLE);
    assert_eq!(dec.period_disagreements, 0, "an on-time host agrees with the PIO");

    // A's delivery is 20 ms late: the host period reads ~86.7 ms against the
    // PIO's true 66.7 ms, a 30% disagreement.
    t += QUARTER + 0.020;
    dec.edge_with_period(Channel::A, t, 0, CYCLE);
    assert_eq!(dec.period_disagreements, 1, "late delivery is detected, not absorbed");
    // And the hardware value is what the decode used, so the threshold was right.
    assert_eq!(dec.hw_period_used, 2);
}

#[test]
fn the_reversal_cost_is_rate_independent() {
    // Geometry, not scheduling: quarter the rate and the cost is identical.
    for divisor in [1.0_f64, 4.0] {
        let cycle = CYCLE * divisor;
        let (q, tq) = (cycle / 4.0, 3.0 * cycle / 4.0);
        let mut dec = RisingEdgeDecoder::new(false);
        let mut t = 0.0;
        for _ in 0..10 {
            dec.edge(Channel::B, t, 0);
            t += q;
            dec.edge(Channel::A, t, 0);
            t += tq;
        }
        // Turnaround, then 10 reverse cycles.
        dec.edge(Channel::A, t, 0);
        t += q;
        for _ in 0..10 {
            dec.edge(Channel::B, t, 0);
            t += tq;
            dec.edge(Channel::A, t, 0);
            t += q;
        }
        assert_eq!(dec.count - (20 - 21), 4, "cost at cycle {cycle}s");
        assert_eq!(dec.ambiguous, 2, "at cycle {cycle}s");
        assert_eq!(dec.missed, 1, "at cycle {cycle}s");
    }
}

#[test]
fn the_backlash_bound_is_documented_as_two_counts() {
    // The separate quantiser error: a channel's rising edge sits at a different
    // physical position depending on direction, so an out-and-back finishes up
    // to one whole tooth pitch (2 counts) high. No decoder can remove it.
    assert_eq!(RisingEdgeDecoder::reversal_backlash_counts(), 2);
}

// ---------------------------------------------------------------------------
// Failure modes
// ---------------------------------------------------------------------------

#[test]
fn a_single_lost_edge_does_not_invert_the_direction() {
    // The regression the Python decoder was fixed for: the "period" across a
    // discontinuity spans three quarter cycles instead of four, which flips the
    // gap comparison unless the timing state is discarded.
    let mut dec = RisingEdgeDecoder::new(false);
    let mut t = drive_forward(&mut dec, 6, 0.0);
    assert_eq!(dec.sense, 1);

    // Drop B's rise: the next edge is A again, one full cycle later.
    t += QUARTER; // where B's rise should have been
    dec.edge(Channel::A, t, 0);
    assert_eq!(dec.missed, 1);
    assert_eq!(dec.sense, 1, "the held sense must survive a lost edge");
    t += THREE_QUARTER;

    // Two more edges are held on the old direction, then it re-measures clean.
    drive_forward(&mut dec, 6, t);
    assert_eq!(dec.sense, 1, "direction must NOT invert after a lost edge");
    assert_eq!(dec.missed, 1);
}

#[test]
fn a_gap_at_half_a_cycle_is_refused_as_ambiguous() {
    // A 50/50 gap split carries no direction information — a mounting that is
    // not 90 degrees, or timestamps mangled in transport (which is exactly what
    // the firmware's 50 ms harvest does).
    let mut dec = RisingEdgeDecoder::new(false);
    // Establish a direction first so we can see the ambiguity rather than a hold.
    let mut t = drive_forward(&mut dec, 4, 0.0);
    let before = dec.ambiguous;

    // Now feed evenly-spaced alternating edges: gap == period/2 exactly.
    let half = CYCLE / 2.0;
    for _ in 0..6 {
        dec.edge(Channel::B, t, 0);
        t += half;
        dec.edge(Channel::A, t, 0);
        t += half;
    }
    assert!(
        dec.ambiguous >= before + 10,
        "evenly-spaced edges must be flagged ambiguous, got {} (was {before})",
        dec.ambiguous
    );
    assert_eq!(dec.sense, 1, "the sign is held, not guessed anew");
}

#[test]
fn the_ambiguity_band_tolerates_45_to_135_degrees_of_phase() {
    // AMBIGUITY_BAND = 0.25 means |2*gap - period| >= period/4 is readable, i.e.
    // gap outside (0.375, 0.625) of a cycle. 45 deg = 0.125, 135 deg = 0.375.
    for (label, frac) in [("45 deg", 0.125_f64), ("135 deg", 0.3749_f64)] {
        let mut dec = RisingEdgeDecoder::new(false);
        let (gap, rest) = (CYCLE * frac, CYCLE * (1.0 - frac));
        let mut t = 0.0;
        for _ in 0..6 {
            dec.edge(Channel::B, t, 0);
            t += gap;
            dec.edge(Channel::A, t, 0);
            t += rest;
        }
        assert_eq!(dec.sense, 1, "{label} must still be readable");
        assert_eq!(dec.ambiguous, 0, "{label} must not be flagged ambiguous");
    }
}

#[test]
fn a_hint_overrides_the_timing_but_leaves_the_state_measurable() {
    let mut dec = RisingEdgeDecoder::new(false);
    // Forward timing, but the commanding controller says we are going backwards.
    dec.edge(Channel::B, 0.0, -1);
    dec.edge(Channel::A, QUARTER, -1);
    assert_eq!(dec.count, -2, "the hint signs both edges immediately");
    assert_eq!(dec.ambiguous, 0, "a hint is a measurement, not a held guess");
    // Drop the hint: the timing state was still maintained, so the next edge
    // measures normally and flips back to the true sense.
    dec.edge(Channel::B, CYCLE, 0);
    assert_eq!(dec.sense, 1, "unhinted edges measure from the real timing");
}

#[test]
fn zero_resets_position_but_keeps_diagnostics() {
    let mut dec = RisingEdgeDecoder::new(false);
    drive_forward(&mut dec, 5, 0.0);
    dec.missed = 3;
    dec.zero();
    assert_eq!(dec.count, 0);
    assert_eq!(dec.missed, 3, "homing does not clear the diagnostics");
    assert_eq!(dec.sense, 1, "nor the learned direction");
}

// ---------------------------------------------------------------------------
// Polled ingest: feed_batch + merge_new_edge_events
// ---------------------------------------------------------------------------

#[test]
fn feed_batch_decodes_rising_and_counts_falling_as_filtered() {
    let mut dec = RisingEdgeDecoder::new(false);
    let ms = |s: f64| (s * 1000.0).round() as i64;
    let mut edges = Vec::new();
    let mut t = 0.0;
    for i in 0..10 {
        edges.push(EdgeRecord {
            event_id: i * 2,
            channel: Channel::B,
            level: true,
            time_ms: ms(t),
        });
        t += QUARTER;
        edges.push(EdgeRecord {
            event_id: i * 2 + 1,
            channel: Channel::A,
            level: true,
            time_ms: ms(t),
        });
        t += THREE_QUARTER;
    }
    // One stray falling edge: a pin still on irq_edge="both".
    edges.push(EdgeRecord { event_id: 999, channel: Channel::A, level: false, time_ms: ms(t) });

    let delta = dec.feed_batch(&edges, |_| 0);
    assert_eq!(dec.count, 20);
    assert_eq!(delta, 20);
    assert_eq!(dec.filtered, 1, "a falling edge must be surfaced, not absorbed");
}

#[test]
fn merge_dedups_by_id_orders_across_pins_and_advances_the_cursor() {
    // The platform re-serves and duplicates events, and event_id is a GLOBAL
    // sequence across pins, so sorting by it recovers true cross-channel order.
    let a = vec![
        RawDiEvent { event_id: 10, event: "DI_R".into(), time_ms: 1000 },
        RawDiEvent { event_id: 12, event: "DI_R".into(), time_ms: 1067 },
        // A duplicate re-serve of an id already taken: first one wins.
        RawDiEvent { event_id: 10, event: "DI_R".into(), time_ms: 1000 },
        // Below the cursor: already decoded.
        RawDiEvent { event_id: 5, event: "DI_R".into(), time_ms: 900 },
        // Not a DI edge at all.
        RawDiEvent { event_id: 13, event: "CM4_ON".into(), time_ms: 1080 },
    ];
    let b = vec![
        RawDiEvent { event_id: 11, event: "DI_R".into(), time_ms: 1017 },
        RawDiEvent { event_id: 14, event: "DI_F".into(), time_ms: 1100 },
    ];

    let (edges, newest) =
        merge_new_edge_events(&[(Channel::A, a.as_slice()), (Channel::B, b.as_slice())], 9);
    assert_eq!(newest, 14, "cursor advances past even non-decodable events");
    assert_eq!(
        edges.iter().map(|e| (e.event_id, e.channel, e.level)).collect::<Vec<_>>(),
        vec![
            (10, Channel::A, true),
            (11, Channel::B, true),
            (12, Channel::A, true),
            (14, Channel::B, false),
        ],
        "ordered by global event_id, interleaving A and B correctly"
    );
}

#[test]
fn the_docker_simulators_event_vocabulary_is_accepted_too() {
    // Real hardware emits DI_R/DI_F; the docker platform-interface simulator
    // emits "rising"/"falling"/"both" instead
    // (platform_iface_sim.py:302-312). The Python app matches only DI_R/DI_F and
    // therefore silently decodes NOTHING against the simulator.
    let a = vec![RawDiEvent { event_id: 1, event: "rising".into(), time_ms: 0 }];
    let b = vec![RawDiEvent { event_id: 2, event: "falling".into(), time_ms: 10 }];
    let (edges, _) =
        merge_new_edge_events(&[(Channel::A, a.as_slice()), (Channel::B, b.as_slice())], 0);
    assert_eq!(edges.len(), 2);
    assert!(edges[0].level && !edges[1].level);
}

// ---------------------------------------------------------------------------
// Granularity and the rate helpers
// ---------------------------------------------------------------------------

#[test]
fn granularity_is_two_counts_per_tooth_cycle() {
    // 2x decode: counts_per_rev is twice the tooth count, and mm_per_count is
    // therefore tooth pitch / 2 — the position granularity in mm.
    assert_eq!(counts_per_rev(16), 32);
    assert_eq!(counts_per_rev(1), 2);
    // A 4 mm tooth pitch gives the shipped default of 2.0 mm per count.
    let mm_per_count = 4.0 / 2.0;
    assert_eq!(mm_per_count, 2.0);
    assert_eq!(revolutions(32, counts_per_rev(16)), 1.0);
    assert_eq!(revolutions(-16, counts_per_rev(16)), -0.5);
    assert_eq!(revolutions(10, 0), 0.0, "a misconfigured cpr must not divide by zero");
}

#[test]
fn travel_and_rotation_words() {
    assert_eq!(travel_direction(3), "opening");
    assert_eq!(travel_direction(-3), "closing");
    assert_eq!(travel_direction(0), "stopped");
    assert_eq!(rotation_direction(10.0, 0.5), "cw");
    assert_eq!(rotation_direction(-10.0, 0.5), "ccw");
    assert_eq!(rotation_direction(0.2, 0.5), "stopped", "jitter must not flicker direction");
}

#[test]
fn rate_estimator_windows_and_resets() {
    let mut rate = RateEstimator::new(60.0);
    assert_eq!(rate.rpm(32), 0.0, "fewer than two samples is 'can't tell yet'");
    for (t, c) in [(100.0, 0), (110.0, 32), (120.0, 64), (130.0, 96)] {
        rate.add(t, c);
    }
    assert_eq!(rate.len(), 4);
    // 96 counts = 3 revs over 30 s = 6 rpm.
    assert_eq!(rate.rpm(32), 6.0);

    rate.window_s = 15.0;
    rate.add(140.0, 128);
    // Only samples newer than 140-15=125 survive: (130, 96) and (140, 128).
    assert_eq!(rate.len(), 2);
    assert_eq!(rate.rpm(32), 6.0);

    rate.reset();
    assert_eq!(rate.rpm(32), 0.0, "a home must not read as a phantom spike");
}
