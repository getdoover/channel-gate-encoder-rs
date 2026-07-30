//! Binary entry point.
//!
//! Three modes:
//!
//! * `channel-gate-encoder` — run the app (the container `ENTRYPOINT`).
//! * `channel-gate-encoder export [path] [--app-name N]` — write the config +
//!   UI schemas into `doover_config.json` without connecting to anything. Handled
//!   by [`doover::run`].
//! * `channel-gate-encoder healthcheck` — probe this container's own healthcheck
//!   endpoint and exit 0/1. The image is `FROM scratch`, so there is no `curl`
//!   for `HEALTHCHECK` to call; the binary answers for itself instead.

use std::io::{Read, Write};
use std::net::TcpStream;
use std::time::Duration;

use channel_gate_encoder::ChannelGateEncoder;
use doover::error::Result;

/// Probe `127.0.0.1:$HEALTHCHECK_PORT` and report whether it answered `200`.
///
/// doover-rs's healthcheck server (`docker/healthcheck.rs`) answers any request
/// with `200 OK` while the app is healthy and `503` once a `main_loop` has
/// failed, so a bare `GET /` is the whole contract.
fn healthcheck() -> std::io::Result<bool> {
    let port: u16 =
        std::env::var("HEALTHCHECK_PORT").ok().and_then(|p| p.parse().ok()).unwrap_or(49200);
    let timeout = Duration::from_secs(2);
    let addr = std::net::SocketAddr::from(([127, 0, 0, 1], port));
    let mut stream = TcpStream::connect_timeout(&addr, timeout)?;
    stream.set_read_timeout(Some(timeout))?;
    stream.set_write_timeout(Some(timeout))?;
    stream.write_all(b"GET / HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")?;
    let mut response = Vec::new();
    // The server closes the connection itself, so read to EOF.
    stream.read_to_end(&mut response)?;
    Ok(String::from_utf8_lossy(&response).starts_with("HTTP/1.1 200"))
}

#[tokio::main]
async fn main() -> Result<()> {
    if std::env::args().nth(1).as_deref() == Some("healthcheck") {
        let ok = healthcheck().unwrap_or(false);
        if !ok {
            eprintln!("healthcheck: app is not healthy");
        }
        std::process::exit(if ok { 0 } else { 1 });
    }

    let level = if std::env::var("DEBUG").is_ok_and(|v| v == "1") {
        tracing::Level::DEBUG
    } else {
        tracing::Level::INFO
    };
    tracing_subscriber::fmt().with_max_level(level).init();
    doover::run::<ChannelGateEncoder>().await
}
