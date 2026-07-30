//! The debug/state endpoint's routing and the direction-hint timeline.
//!
//! `app_e2e.rs` proves `GET /state` serves what was published over a real
//! socket; this covers the mutating routes and the hint semantics, which are
//! the parts the bench motor harness drives.

use std::sync::{Arc, Mutex};

use channel_gate_encoder::core::{hint_at, push_hint};
use channel_gate_encoder::debug_server::{route, DebugHandles};
use channel_gate_encoder::quadrature::Channel;
use channel_gate_encoder::state::{EncoderState, Snapshot};
use serde_json::Value;

fn handles() -> DebugHandles {
    DebugHandles {
        snapshot: Arc::new(Mutex::new(Snapshot::default())),
        state: Arc::new(Mutex::new(EncoderState::new(false))),
        hints: Arc::new(Mutex::new(vec![(0, 0)])),
    }
}

fn request(method: &str, path: &str, body: &str) -> Vec<u8> {
    format!(
        "{method} {path} HTTP/1.1\r\nHost: localhost\r\ncontent-length: {}\r\n\r\n{body}",
        body.len()
    )
    .into_bytes()
}

fn body_of(response: &[u8]) -> Value {
    let text = String::from_utf8_lossy(response);
    let body = text.split("\r\n\r\n").nth(1).unwrap_or("");
    serde_json::from_str(body).unwrap_or(Value::Null)
}

#[test]
fn state_serves_the_last_published_snapshot() {
    let h = handles();
    h.snapshot.lock().unwrap().count = 42;
    h.snapshot.lock().unwrap().height_mm = 84.5;
    let resp = route(&request("GET", "/state", ""), &h);
    assert!(String::from_utf8_lossy(&resp).starts_with("HTTP/1.1 200"));
    let body = body_of(&resp);
    assert_eq!(body["count"].as_i64(), Some(42));
    assert_eq!(body["height_mm"].as_f64(), Some(84.5));
    assert_eq!(body["edge_mode"].as_str(), Some("rising_2x"));
}

#[test]
fn a_query_string_does_not_break_routing() {
    let h = handles();
    let resp = route(&request("GET", "/state?cachebust=1", ""), &h);
    assert!(String::from_utf8_lossy(&resp).starts_with("HTTP/1.1 200"));
}

#[test]
fn zero_homes_the_encoder() {
    let h = handles();
    {
        let mut st = h.state.lock().unwrap();
        st.decoder.edge(Channel::B, 0.0, 1);
        st.decoder.edge(Channel::A, 0.017, 1);
        assert_eq!(st.decoder.count, 2);
        assert!(!st.homed);
    }
    let resp = route(&request("POST", "/zero", ""), &h);
    assert_eq!(body_of(&resp)["homed"].as_bool(), Some(true));
    let st = h.state.lock().unwrap();
    assert_eq!(st.decoder.count, 0);
    assert!(st.homed, "a manual zero counts as homed");
    assert_eq!(st.home_epoch, 1, "the publish path must see the discontinuity");
}

#[test]
fn reset_missed_clears_only_that_counter() {
    let h = handles();
    {
        let mut st = h.state.lock().unwrap();
        st.decoder.missed = 7;
        st.decoder.ambiguous = 3;
    }
    let resp = route(&request("POST", "/reset_missed", ""), &h);
    assert_eq!(body_of(&resp)["missed_edges"].as_i64(), Some(0));
    let st = h.state.lock().unwrap();
    assert_eq!(st.decoder.missed, 0);
    assert_eq!(st.decoder.ambiguous, 3, "ambiguous is a separate failure mode");
}

#[test]
fn direction_hint_accepts_cw_ccw_and_none() {
    let h = handles();
    for (word, expected) in [("cw", 1), ("ccw", -1), ("none", 0), ("CW", 1), ("nonsense", 0)] {
        let resp =
            route(&request("POST", "/direction_hint", &format!(r#"{{"direction":"{word}"}}"#)), &h);
        assert_eq!(body_of(&resp)["direction_hint"].as_i64(), Some(expected), "direction={word}");
    }
    // A malformed body is "none", never an error that could wedge the harness.
    let resp = route(&request("POST", "/direction_hint", "not json"), &h);
    assert_eq!(body_of(&resp)["direction_hint"].as_i64(), Some(0));
}

#[test]
fn an_unknown_route_is_404_not_a_panic() {
    let h = handles();
    let resp = route(&request("GET", "/nope", ""), &h);
    assert!(String::from_utf8_lossy(&resp).starts_with("HTTP/1.1 404"));
    // A truncated/garbage request must also be survivable.
    let resp = route(b"", &h);
    assert!(String::from_utf8_lossy(&resp).starts_with("HTTP/1.1 404"));
}

#[test]
fn hints_are_applied_at_capture_time_not_decode_time() {
    // The platform syncs DI events 30-90 s late, so a hint must be looked up
    // against the event's CAPTURE timestamp or stragglers get mis-signed.
    let timeline = vec![(0, 0), (1_000, 1), (5_000, -1), (9_000, 0)];
    assert_eq!(hint_at(&timeline, 500), 0, "before any hint");
    assert_eq!(hint_at(&timeline, 1_000), 1, "at the boundary the new hint applies");
    assert_eq!(hint_at(&timeline, 4_999), 1);
    assert_eq!(hint_at(&timeline, 5_000), -1);
    assert_eq!(hint_at(&timeline, 100_000), 0, "the last state persists");
}

#[test]
fn the_hint_timeline_is_trimmed_but_never_emptied() {
    let hints = Mutex::new(vec![(0, 0)]);
    for _ in 0..50 {
        push_hint(&hints, 1);
    }
    let timeline = hints.lock().unwrap();
    // Everything just pushed is inside the 10-minute window, so nothing is
    // dropped yet — but the seed entry plus every push must be present and the
    // latest state readable.
    assert!(timeline.len() >= 2, "the timeline must retain its history: {timeline:?}");
    assert_eq!(timeline.last().unwrap().1, 1);
}
