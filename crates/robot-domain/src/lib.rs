//! Hardware- and middleware-independent robot concepts.
//!
//! This crate models the capabilities that the current Pico protocol can
//! express. In particular, motion is directional today; velocity commands and
//! localization values will be added when a device and protocol can support
//! them end to end.

use std::time::Instant;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionState {
    Stopped,
    Forward,
    Backward,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MotionCommand {
    SetDirection(Direction),
    Stop,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Capability {
    Motor,
    Encoder,
    Camera,
    Lidar,
    Imu,
    Gnss,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Health {
    Ready,
    Degraded(String),
    Disconnected,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ControlMode {
    Manual,
    Autonomous,
    SafeStop,
}

impl ControlMode {
    /// Whether a source may propose ordinary motion commands in this mode.
    pub fn accepts(self, source: CommandSource) -> bool {
        matches!(
            (self, source),
            (Self::Manual, CommandSource::Cli | CommandSource::Teleop)
                | (
                    Self::Autonomous,
                    CommandSource::Nav2 | CommandSource::Autoware | CommandSource::Mission
                )
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandSource {
    Cli,
    Teleop,
    Nav2,
    Autoware,
    Mission,
    Safety,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ResourceId {
    DriveBase,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlLease {
    pub id: u64,
    pub owner: CommandSource,
    pub expires_at: Instant,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlState {
    pub mode: ControlMode,
    pub active_lease: Option<ControlLease>,
}

impl Default for ControlState {
    fn default() -> Self {
        Self {
            mode: ControlMode::Manual,
            active_lease: None,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct SafetyState {
    pub latched: bool,
    pub reason: Option<String>,
    /// Increases on each new stop event so a concurrent reset cannot erase it.
    pub generation: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RobotState {
    pub motion: MotionState,
}

impl Default for MotionState {
    fn default() -> Self {
        Self::Stopped
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct StateSnapshot {
    pub version: u64,
    pub robot: RobotState,
    pub control: ControlState,
    pub safety: SafetyState,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CommandProposal {
    pub source: CommandSource,
    pub resource: ResourceId,
    pub command: MotionCommand,
    pub issued_at: Instant,
    pub valid_until: Instant,
    pub control_lease_id: Option<u64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RejectionReason {
    NoAuthority,
    Expired,
    ResourceBusy,
    Unsafe,
    InvalidMode,
}
