//! Published tags — a port of `src/channel_gate_encoder/app_tags.py`.
//!
//! Everything marked `live` is published for other apps to consume — read from
//! a peer app with, e.g. `ctx.get_remote_tag("channel_gate_encoder_1",
//! "Height")` (mm).
//!
//! `Height` (mm) is the headline output; the rest give percent-open, movement,
//! homing status and diagnostics. `Heartbeat` updates every publish so
//! consumers can tell a live feed from a frozen one, and `Homed` tells them
//! whether the absolute height can be trusted yet.
//!
//! `MissedEdges` and `AmbiguousEdges` are the two ways a rising-edge-only
//! encoder goes wrong, and they are different: a missed edge is a rise that
//! never arrived (position too low), an ambiguous edge is a rise that arrived
//! but whose DIRECTION could not be measured, so it was signed by the last
//! known direction (position may have moved the wrong way). Either one growing
//! during motion means the height should not be trusted until the next home.
//!
//! The tag **names** are the field names, so they match the Python
//! declarations exactly and a peer app reading `Height` keeps working.

use doover::{Tag, Tags};

#[allow(non_snake_case)]
#[derive(Tags)]
pub struct ChannelGateEncoderTags {
    /// Gate height, mm.
    #[tag(live, default = 0.0)]
    pub Height: Tag<f64>,
    /// Percent of gate travel.
    #[tag(live, default = 0.0)]
    pub PercentOpen: Tag<f64>,
    /// Signed rising edges (2x decode).
    #[tag(live, default = 0)]
    pub RawCount: Tag<i64>,
    /// Whether the count has been zeroed against the reference.
    #[tag(live, default = false)]
    pub Homed: Tag<bool>,
    /// opening / closing / stopped.
    #[tag(live, default = "stopped")]
    pub Direction: Tag<String>,
    /// mm/min.
    #[tag(live, default = 0.0)]
    pub Speed: Tag<f64>,
    /// Limit switch asserted.
    #[tag(live, default = false)]
    pub HomeSwitch: Tag<bool>,
    /// Same-channel repeats.
    #[tag(live, default = 0)]
    pub MissedEdges: Tag<i64>,
    /// Counts of unknown sign.
    #[tag(live, default = 0)]
    pub AmbiguousEdges: Tag<i64>,
    /// Epoch seconds; freshness.
    #[tag(live, default = 0.0)]
    pub Heartbeat: Tag<f64>,

    // Rotation readouts (target-wheel motion, independent of gate geometry).
    /// Signed revs since zero.
    #[tag(live, default = 0.0)]
    pub Revolutions: Tag<f64>,
    /// Signed rpm; + = CW.
    #[tag(live, default = 0.0)]
    pub RPM: Tag<f64>,
    /// cw / ccw / stopped.
    #[tag(live, default = "stopped")]
    pub RotationDirection: Tag<String>,
}
