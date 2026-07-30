//! Cloud UI — a port of `src/channel_gate_encoder/app_ui.py`.
//!
//! The headline is the gate height; below it sit percent-open, movement
//! direction/speed, homing status and diagnostics. All readouts are bound to
//! the live tags, so setting the tag updates the display. Two buttons let an
//! operator zero the encoder in the field.
//!
//! Element names are pinned with `.name(...)` to match the Python export
//! exactly (pydoover derives the name from the display string, which would
//! otherwise give `gate_height_mm` instead of `height`) — an existing dashboard
//! and the `ui_cmds` handler names both depend on them.

use doover::ui::{
    BooleanVariable, Button, NumericVariable, TextVariable, UiBuild, WarningIndicator,
};
use doover::Ui;

use crate::tags::ChannelGateEncoderTags;

#[derive(Ui)]
pub struct ChannelGateEncoderUi {
    pub height: NumericVariable,
    pub percent_open: NumericVariable,
    pub direction: TextVariable,
    pub speed: NumericVariable,

    // Rotation readouts (target-wheel motion).
    pub revolutions: NumericVariable,
    pub rpm: NumericVariable,
    pub rotation: TextVariable,

    pub homed: BooleanVariable,
    pub not_homed_warning: WarningIndicator,
    pub home_switch: BooleanVariable,

    // Diagnostics
    pub raw_count: NumericVariable,
    pub missed_edges: NumericVariable,

    // Field actions (interaction names == command handler names)
    pub set_home: Button,
    pub reset_missed: Button,
}

impl UiBuild for ChannelGateEncoderUi {
    type Tags = ChannelGateEncoderTags;

    fn build(tags: &ChannelGateEncoderTags) -> Self {
        Self {
            height: NumericVariable::new("Gate Height (mm)")
                .name("height")
                .precision(1)
                .value(&tags.Height),
            percent_open: NumericVariable::new("Percent Open (%)")
                .name("percent_open")
                .precision(1)
                .value(&tags.PercentOpen),
            direction: TextVariable::new("Movement").name("direction").value(&tags.Direction),
            speed: NumericVariable::new("Speed (mm/min)")
                .name("speed")
                .precision(0)
                .value(&tags.Speed),
            revolutions: NumericVariable::new("Revolutions")
                .name("revolutions")
                .precision(2)
                .value(&tags.Revolutions),
            rpm: NumericVariable::new("RPM").name("rpm").precision(1).value(&tags.RPM),
            rotation: TextVariable::new("Rotation").name("rotation").value(&tags.RotationDirection),
            homed: BooleanVariable::new("Homed").name("homed").value(&tags.Homed),
            not_homed_warning: WarningIndicator::new("Not homed - height unverified")
                .name("not_homed")
                .hidden(true)
                .can_cancel(false),
            home_switch: BooleanVariable::new("At Home Switch")
                .name("home_switch")
                .value(&tags.HomeSwitch),
            raw_count: NumericVariable::new("Raw Count")
                .name("raw_count")
                .precision(0)
                .value(&tags.RawCount),
            missed_edges: NumericVariable::new("Missed Edges")
                .name("missed_edges")
                .precision(0)
                .value(&tags.MissedEdges),
            set_home: Button::new("Set Home Here").name("set_home").colour("blue"),
            reset_missed: Button::new("Clear Missed Count").name("reset_missed"),
        }
    }
}
