use crate::ports::outbound::ControlError;
use async_trait::async_trait;
use robot_domain::{ControlLease, Direction, MotionState};
use std::time::Duration;

/// Application entry point used by CLI, teleoperation, and future ROS bridges.
#[async_trait]
pub trait DriveControl: Send + Sync {
    async fn acquire_control(&self, ttl: Duration) -> Result<ControlLease, ControlError>;

    async fn renew_control(
        &self,
        lease: ControlLease,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError>;

    async fn release_control(&self, lease: ControlLease) -> Result<(), ControlError>;
    async fn drive(
        &self,
        lease: ControlLease,
        direction: Direction,
    ) -> Result<MotionState, ControlError>;
    async fn stop(&self, lease: ControlLease) -> Result<MotionState, ControlError>;
    async fn emergency_stop(&self, reason: String) -> Result<MotionState, ControlError>;
    async fn reset_safety_stop(&self) -> Result<(), ControlError>;
    async fn status(&self) -> Result<MotionState, ControlError>;
}
