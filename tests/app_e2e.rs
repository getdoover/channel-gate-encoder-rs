//! End-to-end: the **real** `doover::run_with` runner driving the real
//! `ChannelGateEncoder`, against a fake device agent and the gRPC platform
//! double.
//!
//! Where `fidelity.rs` measures the pulse path in isolation, this proves the
//! whole contract the device actually depends on: the app connects to the
//! platform interface, asks the firmware for rising-only interrupts, publishes to
//! `tag_values` under the right tag names at the configured cadence, serves its
//! debug endpoint, and answers its own healthcheck.

mod common;

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use common::fake_agent::{spawn_fake_agent, FakeAgentState};
use common::platform_double::{spawn_platform_double, QuadratureGate};
use serde_json::{json, Value};

use channel_gate_encoder::ChannelGateEncoder;
use doover::RunOptions;

const APP_KEY: &str = "channel_gate_encoder_1";
const A_PIN: i32 = 0;
const B_PIN: i32 = 1;

/// Every value the app has ever written for `tag_values.<APP_KEY>.<name>`.
fn tag_writes(state: &FakeAgentState, name: &str) -> Vec<Value> {
    state
        .aggregate_writes
        .lock()
        .unwrap()
        .iter()
        .filter(|w| w.channel == "tag_values")
        .filter_map(|w| w.data.get(APP_KEY).and_then(|a| a.get(name)).cloned())
        .collect()
}

async fn wait_for(what: &str, mut predicate: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(20);
    while !predicate() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        tokio::time::sleep(Duration::from_millis(25)).await;
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 4)]
async fn the_real_runner_publishes_position_to_tags_at_the_configured_cadence() {
    let (agent, dda_uri) = spawn_fake_agent().await;
    agent.seed_aggregate("tag_values", json!({}));
    agent.seed_aggregate("dv-ui-sub", json!({}));
    let (plt_state, plt_uri) = spawn_platform_double().await;

    // The app reads PLT_URI from the environment (as the level_sensor example
    // does), so point it at the double. Stripped of its scheme, since the app
    // prefixes http://.
    std::env::set_var("PLT_URI", plt_uri.trim_start_matches("http://"));

    // A deployment config with streaming ingest, a 200 ms publish interval and
    // 10 mm per count so position changes are unmistakable.
    let mut config_path = std::env::temp_dir();
    config_path.push(format!("cge-rs-e2e-{}.json", std::process::id()));
    std::fs::write(
        &config_path,
        json!({
            "channel_a_pin": A_PIN,
            "channel_b_pin": B_PIN,
            "mm_per_count": 10.0,
            "pulses_per_rev": 16,
            "use_event_polling": false,
            "tag_publish_interval_s": 0.2,
            "display_refresh_period_s": 0.2,
            "gate_travel_mm": 1000.0,
            "debounce_ms": 3,
            "APP_KEY": APP_KEY,
            "APP_DISPLAY_NAME": "Channel Gate Encoder"
        })
        .to_string(),
    )
    .unwrap();

    let healthcheck_port = 49411;
    let opts = RunOptions {
        dda_uri,
        plt_uri: plt_uri.clone(),
        modbus_uri: String::new(),
        app_key: APP_KEY.to_string(),
        config_fp: Some(config_path.to_string_lossy().into_owned()),
        healthcheck_port,
        debug: false,
        error_wait: Duration::from_secs(1),
    };
    let runner = tokio::spawn(doover::run_with::<ChannelGateEncoder>(opts));

    // Setup is done once both rising subscriptions are registered.
    wait_for("both pulse subscriptions", || plt_state.subscriber_count() >= 2).await;

    // The app must have asked the FIRMWARE for rising-only interrupts, not just
    // filtered at the subscription. This is load-bearing: with the pin left on
    // "both", the firmware's 50 ms harvest majority-votes a rise and a fall in
    // one window into a FALLING event and deletes the rise (measured upstream at
    // 68% of rising edges destroyed).
    {
        let edges = plt_state.di_irq_edge.lock().unwrap();
        assert_eq!(edges.get(&A_PIN).map(String::as_str), Some("rising"), "pin A irq_edge");
        assert_eq!(edges.get(&B_PIN).map(String::as_str), Some("rising"), "pin B irq_edge");
        let debounce = plt_state.di_debounce_ms.lock().unwrap();
        assert_eq!(debounce.get(&A_PIN).copied(), Some(3), "configured debounce reaches the pin");
    }

    // The healthcheck endpoint the container HEALTHCHECK probes.
    wait_for("healthcheck to report healthy", || {
        probe_healthcheck(healthcheck_port).unwrap_or(false)
    })
    .await;

    // Drive the gate for 2 s at the specified rate.
    let mut gate = QuadratureGate::new(plt_state.clone(), A_PIN, B_PIN, 15.0);
    gate.seed();
    gate.run_for(Duration::from_secs(2), None).await;

    wait_for("a RawCount tag write", || !tag_writes(&agent, "RawCount").is_empty()).await;
    // Give the publish timer a couple more passes so the last edges land.
    tokio::time::sleep(Duration::from_millis(600)).await;

    let counts = tag_writes(&agent, "RawCount");
    let heights = tag_writes(&agent, "Height");
    let directions = tag_writes(&agent, "Direction");
    let cpr_revs = tag_writes(&agent, "Revolutions");

    let final_count = counts.last().and_then(Value::as_i64).expect("a RawCount value");
    assert_eq!(
        final_count,
        gate.true_position,
        "the published count must equal the gate truth ({} tag writes)",
        counts.len()
    );
    assert_eq!(
        heights.last().and_then(Value::as_f64),
        Some(final_count as f64 * 10.0),
        "Height = count * mm_per_count"
    );
    // 2 s at a 200 ms publish interval is ~10 publishes, and must be nowhere near
    // the 60 rising edges that arrived.
    assert!(
        (5..=15).contains(&counts.len()),
        "publish cadence follows tag_publish_interval_s, not the pulse rate: {} writes for {} edges",
        counts.len(),
        gate.rising_emitted
    );
    // Direction comes from the count delta, with no drive information at all —
    // no DO state, no valve knowledge, only the A/B rise timing.
    assert!(
        directions.iter().any(|v| v.as_str() == Some("opening")),
        "must have read 'opening' while the gate was moving: {directions:?}"
    );
    // And it must not LATCH: once the edges stop, the next publish reads
    // stopped, because count_delta is zero.
    assert_eq!(
        directions.last().and_then(|v| v.as_str()),
        Some("stopped"),
        "direction must not latch after motion ends: {directions:?}"
    );
    // 2x decode: counts_per_rev = 2 * 16 = 32.
    assert_eq!(
        cpr_revs.last().and_then(Value::as_f64),
        Some((final_count as f64 / 32.0 * 10_000.0).round() / 10_000.0),
        "Revolutions uses the 2x counts_per_rev"
    );

    // The debug endpoint serves exactly what was last published.
    let body = probe_state(channel_gate_encoder::debug_server::PORT).expect("GET /state");
    let served: Value = serde_json::from_str(&body).expect("/state is JSON");
    assert_eq!(served["count"].as_i64(), Some(final_count), "/state matches the tags");
    assert_eq!(served["edge_mode"].as_str(), Some("rising_2x"));
    assert_eq!(served["input_mode"].as_str(), Some("stream"));
    assert_eq!(served["publish_interval_s"].as_f64(), Some(0.2));
    assert_eq!(served["counts_per_rev"].as_i64(), Some(32));
    assert_eq!(
        served["pulses_reporting_high"].as_i64(),
        Some(0),
        "upstream bug 1: no pulse ever carries a level"
    );
    assert_eq!(served["filtered_edges"].as_i64(), Some(0));

    assert_eq!(plt_state.queue_drops.load(Ordering::Relaxed), 0, "the double lost nothing");
    runner.abort();
    let _ = std::fs::remove_file(&config_path);
}

/// The same probe the binary's `healthcheck` subcommand performs.
fn probe_healthcheck(port: u16) -> std::io::Result<bool> {
    Ok(http_get(port, "/")?.starts_with("HTTP/1.1 200"))
}

/// `GET /state` from the debug endpoint, returning just the body.
fn probe_state(port: u16) -> std::io::Result<String> {
    let raw = http_get(port, "/state")?;
    Ok(raw.split("\r\n\r\n").nth(1).unwrap_or("").to_string())
}

fn http_get(port: u16, path: &str) -> std::io::Result<String> {
    use std::io::{Read, Write};
    let addr = std::net::SocketAddr::from(([127, 0, 0, 1], port));
    let mut stream = std::net::TcpStream::connect_timeout(&addr, Duration::from_secs(2))?;
    stream.set_read_timeout(Some(Duration::from_secs(2)))?;
    write!(stream, "GET {path} HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")?;
    let mut out = Vec::new();
    stream.read_to_end(&mut out)?;
    Ok(String::from_utf8_lossy(&out).into_owned())
}
