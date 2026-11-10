use robot_domain::{ControlState, Health, MotionCommand, MotionState, SafetyState, StateSnapshot};
use std::{error::Error, fmt, time::Instant};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ControlError {
    Disconnected,
    Timeout,
    Rejected,
    NoAuthority,
    ResourceBusy,
    Expired,
    InvalidMode,
    Unsafe,
    LeaseNotExtended,
    DeviceUnavailable(String),
    InvalidLease,
    InvalidDuration,
    StateConflict,
    Transport(String),
}

impl fmt::Display for ControlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Disconnected => write!(f, "device disconnected"),
            Self::Timeout => write!(f, "command timed out"),
            Self::Rejected => write!(f, "device rejected command"),
            Self::NoAuthority => write!(f, "command source has no control authority"),
            Self::ResourceBusy => write!(f, "control resource is owned by another source"),
            Self::Expired => write!(f, "command or control lease expired"),
            Self::InvalidMode => write!(f, "command is not allowed in the current control mode"),
            Self::Unsafe => write!(f, "safety stop is latched"),
            Self::LeaseNotExtended => write!(f, "lease renewal must extend the active lease"),
            Self::DeviceUnavailable(reason) => write!(f, "device is unavailable: {reason}"),
            Self::InvalidLease => write!(f, "control lease is invalid"),
            Self::InvalidDuration => write!(f, "duration must be positive and representable"),
            Self::StateConflict => write!(f, "robot state changed while processing the request"),
            Self::Transport(s) => write!(f, "transport error: {s}"),
        }
    }
}

impl Error for ControlError {}

/// Outbound port for a motion device. A successful `stop` or
/// `emergency_stop` must include a fresh acknowledgement that the device is
/// stopped; an acknowledgement with a moving state is an error.
pub trait MotorPort: Send + Sync {
    fn command(
        &self,
        command: MotionCommand,
        lease_id: u64,
        valid_until: Instant,
    ) -> Result<MotionState, ControlError>;

    fn renew_lease(&self, lease_id: u64, valid_until: Instant) -> Result<(), ControlError>;

    /// Sends STOP and confirms the returned device state before succeeding.
    fn stop(&self) -> Result<MotionState, ControlError>;

    /// Latches the adapter's stop state and confirms a fresh STOP response.
    fn emergency_stop(&self) -> Result<MotionState, ControlError>;
    fn clear_emergency_stop(&self) -> Result<(), ControlError>;
    fn read_state(&self) -> Result<MotionState, ControlError>;
}

pub trait RobotStateReader: Send + Sync {
    fn snapshot(&self) -> StateSnapshot;
}

/// Narrow writer ports keep each state component's update responsibility clear.
pub trait RobotMotionStateWriter: Send + Sync {
    fn report_motion_if_safety_clear(&self, state: MotionState, generation: u64) -> bool;

    fn report_motion_if_safety_latched(&self, state: MotionState, generation: u64) -> bool;
}

pub trait ControlStateWriter: Send + Sync {
    fn compare_and_set(&self, expected: ControlState, next: ControlState) -> bool;
    fn compare_and_set_if_safety_clear(
        &self,
        expected: ControlState,
        next: ControlState,
        safety_generation: u64,
    ) -> bool;
    fn force_safe_stop(&self);
}

pub trait SafetyStateWriter: Send + Sync {
    fn latch(&self, reason: String) -> SafetyState;
    fn clear_if_generation(&self, generation: u64) -> bool;
}

pub trait DeviceHealthReader: Send + Sync {
    fn health(&self, device_id: &str) -> Option<Health>;
}
