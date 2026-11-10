use crate::ports::outbound::ControlError;
use robot_domain::{ControlLease, Direction, MotionState};
use std::time::Duration;

/// Application entry point used by CLI, teleoperation, and future ROS bridges.
pub trait DriveControl: Send + Sync {
    fn acquire_control(&self, ttl: Duration) -> Result<ControlLease, ControlError>;

    fn renew_control(
        &self,
        lease: ControlLease,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError>;

    fn release_control(&self, lease: ControlLease) -> Result<(), ControlError>;
    fn drive(&self, lease: ControlLease, direction: Direction)
    -> Result<MotionState, ControlError>;
    fn stop(&self, lease: ControlLease) -> Result<MotionState, ControlError>;
    fn emergency_stop(&self, reason: String) -> Result<MotionState, ControlError>;
    fn reset_safety_stop(&self) -> Result<(), ControlError>;
    fn status(&self) -> Result<MotionState, ControlError>;
}
