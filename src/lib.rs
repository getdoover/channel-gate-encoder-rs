//! Channel gate encoder — a Rust rewrite of the Python `channel-gate-encoder`
//! device app on top of [`doover`] (doover-rs 0.1.3).
//!
//! Two proximity sensors watch a toothed target driven by a channel gate,
//! mounted a quarter-pitch apart. The app subscribes to the **rising edge of
//! each pin, one subscription per pin**, decodes 2x (two counts per tooth
//! cycle), maintains an absolute position homed against a limit switch, and
//! publishes position + direction to tags on a configurable timer (default
//! 500 ms) that is completely decoupled from the edge rate.
//!
//! ## Shape of the thing
//!
//! ```text
//!  platform interface (gRPC :50053)
//!        │  startPulseCounter(di=A, edge="rising")   15 rising/s
//!        │  startPulseCounter(di=B, edge="rising")   15 rising/s
//!        ▼
//!  subscribe_di_pulses → raw stream           (one task per pin)
//!        │  synchronous body, no awaits, no dt<=0 filter
//!        ▼
//!  EncoderState { RisingEdgeDecoder, counters }   ← Mutex, ~100 ns per pulse
//!        ▲
//!        │  read once per publish
//!  EncoderCore::publish(now) → Snapshot → tags + /state
//!        ▲
//!        │  gated at tag_publish_interval_s
//!  Application::main_loop  (runner ticks it, then commit_tags())
//! ```
//!
//! ## What rising-only costs
//!
//! Everything about the decode is a consequence of having no level information
//! and no falling edges: see [`quadrature`] for the direction test, the
//! ambiguity band, the 4-count reversal cost and the irreducible
//! direction-hysteresis of rising-edge sensing. Read that module before changing
//! anything here.

pub mod app;
pub mod config;
pub mod core;
pub mod debug_server;
pub mod quadrature;
pub mod state;
pub mod tags;
pub mod ui;

pub use app::ChannelGateEncoder;
pub use config::ChannelGateEncoderConfig;
pub use core::{EncoderCore, Params};
pub use quadrature::{Channel, RisingEdgeDecoder};
pub use tags::ChannelGateEncoderTags;
pub use ui::ChannelGateEncoderUi;
