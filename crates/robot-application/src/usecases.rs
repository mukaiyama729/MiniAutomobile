use crate::ports::{ControlError, MotorPort};
use robot_domain::{Direction, MotionState};
use std::sync::Arc;

#[derive(Clone)]
pub struct DriveUseCase { motor: Arc<dyn MotorPort> }
impl DriveUseCase {
    pub fn new(motor: Arc<dyn MotorPort>) -> Self { Self { motor } }
    pub fn drive(&self, direction: Direction) -> Result<MotionState, ControlError> {
        self.motor.set_direction(direction)
    }
    pub fn stop(&self) -> Result<MotionState, ControlError> { self.motor.stop() }
    pub fn status(&self) -> Result<MotionState, ControlError> { self.motor.read_state() }
}

#[cfg(test)]
mod tests {
    use super::*;
    struct FakeMotor;
    impl MotorPort for FakeMotor {
        fn set_direction(&self, direction: Direction) -> Result<MotionState, ControlError> {
            Ok(match direction { Direction::Forward => MotionState::Forward, Direction::Backward => MotionState::Backward })
        }
        fn stop(&self) -> Result<MotionState, ControlError> { Ok(MotionState::Stopped) }
        fn read_state(&self) -> Result<MotionState, ControlError> { Ok(MotionState::Stopped) }
    }
    #[test]
    fn usecase_can_run_without_usb_or_ros() {
        let drive = DriveUseCase::new(Arc::new(FakeMotor));
        assert_eq!(drive.drive(Direction::Forward).unwrap(), MotionState::Forward);
        assert_eq!(drive.stop().unwrap(), MotionState::Stopped);
    }
}
