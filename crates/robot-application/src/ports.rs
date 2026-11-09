use robot_domain::{Direction, MotionState};
use std::{error::Error, fmt};

#[derive(Debug)]
pub enum ControlError {
    Disconnected,
    Timeout,
    Rejected,
    Transport(String),
}
impl fmt::Display for ControlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Disconnected => write!(f, "device disconnected"),
            Self::Timeout => write!(f, "command timed out"),
            Self::Rejected => write!(f, "device rejected command"),
            Self::Transport(s) => write!(f, "transport error: {s}"),
        }
    }
}
impl Error for ControlError {}

pub trait MotorPort: Send + Sync {
    fn set_direction(&self, direction: Direction) -> Result<MotionState, ControlError>;
    fn stop(&self) -> Result<MotionState, ControlError>;
    fn read_state(&self) -> Result<MotionState, ControlError>;
}
