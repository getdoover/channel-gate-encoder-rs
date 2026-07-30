//! Localhost-only debug/state endpoint — a port of the Python app's aiohttp
//! server (`application.py:519 _start_dev_endpoint`).
//!
//! Serves **exactly what the app last published**, read out of the in-memory
//! [`Snapshot`](crate::state::Snapshot) rather than by reading tags back, which
//! is what makes it useful for on-device verification: if `/state` and the cloud
//! tags disagree, the problem is the tag path, not the decode.
//!
//! Routes (same shapes as the Python version, so the bench motor harness needs
//! no changes):
//!
//! | | |
//! |---|---|
//! | `GET  /state`          | the last published snapshot as JSON |
//! | `GET  /healthz`        | `OK` — a liveness probe for the endpoint itself |
//! | `POST /zero`           | same effect as the Set Home button |
//! | `POST /reset_missed`   | clear the missed-edge diagnostic counter |
//! | `POST /direction_hint` | `{"direction": "cw"\|"ccw"\|"none"}` |
//!
//! Implemented as a hand-rolled HTTP/1.1 responder over a raw `TcpListener`,
//! mirroring `doover::docker::healthcheck` — the app image is a `scratch`
//! container with a static binary, so pulling an HTTP framework in for four
//! routes would be the wrong trade.

use std::sync::{Arc, Mutex};

use serde_json::json;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;

use crate::core::push_hint;
use crate::state::{EncoderState, Snapshot};

/// Localhost-only, so it is never reachable off-device.
pub const HOST: &str = "127.0.0.1";
pub const PORT: u16 = 8765;

/// Everything the endpoint needs; all shared with the running app.
#[derive(Clone)]
pub struct DebugHandles {
    pub snapshot: Arc<Mutex<Snapshot>>,
    pub state: Arc<Mutex<EncoderState>>,
    pub hints: Arc<Mutex<Vec<(i64, i8)>>>,
}

/// Start the endpoint in the background. Best effort: a bind failure is logged
/// and swallowed so it can never take the encoder down (the Python app wraps the
/// whole thing in a `try`/`except` for the same reason).
pub async fn spawn(port: u16, handles: DebugHandles) {
    let listener = match TcpListener::bind((HOST, port)).await {
        Ok(l) => {
            tracing::info!("Debug state endpoint on http://{HOST}:{port}");
            l
        }
        Err(e) => {
            tracing::warn!("Debug state endpoint not started: {e}");
            return;
        }
    };
    tokio::spawn(async move {
        loop {
            let Ok((mut socket, _)) = listener.accept().await else { continue };
            let handles = handles.clone();
            tokio::spawn(async move {
                // Read until the headers AND the Content-Length body have both
                // arrived. A single read() races clients that write headers and
                // body separately (python urllib does) — the body is then
                // missed and a /direction_hint quietly parses as "none", which
                // mis-signs every edge of the following move.
                let mut buf = Vec::with_capacity(2048);
                let mut chunk = [0u8; 2048];
                let deadline = tokio::time::Instant::now() + std::time::Duration::from_secs(2);
                loop {
                    let n = match tokio::time::timeout_at(deadline, socket.read(&mut chunk)).await
                    {
                        Ok(Ok(n)) => n,
                        _ => 0,
                    };
                    if n == 0 {
                        break;
                    }
                    buf.extend_from_slice(&chunk[..n]);
                    if request_complete(&buf) || buf.len() > 64 * 1024 {
                        break;
                    }
                }
                let response = route(&buf, &handles);
                let _ = socket.write_all(&response).await;
                let _ = socket.shutdown().await;
            });
        }
    });
}

/// Whether `raw` holds a complete request: headers terminated, and at least
/// `Content-Length` bytes of body after them (0 when the header is absent).
fn request_complete(raw: &[u8]) -> bool {
    let Some(head_end) = raw.windows(4).position(|w| w == b"\r\n\r\n") else {
        return false;
    };
    let head = String::from_utf8_lossy(&raw[..head_end]);
    let content_length = head
        .lines()
        .find_map(|l| {
            let (name, value) = l.split_once(':')?;
            name.eq_ignore_ascii_case("content-length").then(|| value.trim().parse::<usize>())
        })
        .and_then(Result::ok)
        .unwrap_or(0);
    raw.len() >= head_end + 4 + content_length
}

/// Dispatch one raw request. Split out from the socket handling so it is
/// directly unit-testable.
pub fn route(raw: &[u8], handles: &DebugHandles) -> Vec<u8> {
    let text = String::from_utf8_lossy(raw);
    let mut parts = text.split_whitespace();
    let method = parts.next().unwrap_or("");
    let path = parts.next().unwrap_or("");
    // Strip any query string; none of these routes take parameters.
    let path = path.split('?').next().unwrap_or("");
    let body = text.split("\r\n\r\n").nth(1).unwrap_or("");

    match (method, path) {
        ("GET", "/state") => {
            json_response(200, &handles.snapshot.lock().expect("snapshot lock").to_json())
        }
        ("GET", "/healthz") => json_response(200, &json!({"ok": true})),
        ("POST", "/zero") => {
            handles.state.lock().expect("state lock").home();
            json_response(200, &json!({"homed": true}))
        }
        ("POST", "/reset_missed") => {
            handles.state.lock().expect("state lock").decoder.missed = 0;
            json_response(200, &json!({"missed_edges": 0}))
        }
        // Polled-ingest bench affordance: the commanding controller declares
        // which way it is driving, and the change is timestamped so late-synced
        // events are signed by the hint active when they were CAPTURED. This is
        // a diagnostic crutch for measuring against a known-good direction, NOT
        // the shipping answer — with no hint the decoder infers direction from
        // the rise timing.
        ("POST", "/direction_hint") => {
            let word = serde_json::from_str::<serde_json::Value>(body)
                .ok()
                .and_then(|v| v.get("direction").and_then(|d| d.as_str()).map(str::to_lowercase))
                .unwrap_or_else(|| "none".to_string());
            let hint: i8 = match word.as_str() {
                "cw" => 1,
                "ccw" => -1,
                _ => 0,
            };
            push_hint(&handles.hints, hint);
            json_response(200, &json!({"direction_hint": hint}))
        }
        _ => json_response(404, &json!({"error": "not found"})),
    }
}

fn json_response(status: u16, body: &serde_json::Value) -> Vec<u8> {
    let body = body.to_string();
    let reason = if status == 200 { "OK" } else { "Not Found" };
    format!(
        "HTTP/1.1 {status} {reason}\r\ncontent-type: application/json\r\ncontent-length: {}\r\nconnection: close\r\n\r\n{body}",
        body.len()
    )
    .into_bytes()
}
