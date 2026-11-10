use crate::ports::outbound::{
    ControlError, ControlStateWriter, DeviceHealthReader, MotorPort, RobotMotionStateWriter,
    RobotStateReader, SafetyStateWriter,
};
use robot_domain::{ControlMode, ControlState, Health, MotionState};
use std::sync::Arc;

/// Owns the application safety latch. Emergency stop bypasses the ordinary
/// command gateway; clearing the latch requires an explicit, checked reset.
pub struct SafetySupervisor {
    state_reader: Arc<dyn RobotStateReader>,
    control_writer: Arc<dyn ControlStateWriter>,
    safety_writer: Arc<dyn SafetyStateWriter>,
    motion_writer: Arc<dyn RobotMotionStateWriter>,
    motor: Arc<dyn MotorPort>,
    health: Arc<dyn DeviceHealthReader>,
    motor_id: String,
}

impl SafetySupervisor {
    pub fn new(
        state_reader: Arc<dyn RobotStateReader>,
        control_writer: Arc<dyn ControlStateWriter>,
        safety_writer: Arc<dyn SafetyStateWriter>,
        motion_writer: Arc<dyn RobotMotionStateWriter>,
        motor: Arc<dyn MotorPort>,
        health: Arc<dyn DeviceHealthReader>,
        motor_id: impl Into<String>,
    ) -> Self {
        Self {
            state_reader,
            control_writer,
            safety_writer,
            motion_writer,
            motor,
            health,
            motor_id: motor_id.into(),
        }
    }

    pub fn emergency_stop(&self, reason: String) -> Result<MotionState, ControlError> {
        let latched = self.safety_writer.latch(reason);
        self.control_writer.force_safe_stop();
        match self.motor.emergency_stop() {
            Ok(state) => {
                self.motion_writer
                    .report_motion_if_safety_latched(state, latched.generation);
                if state == MotionState::Stopped {
                    Ok(state)
                } else {
                    Err(ControlError::Unsafe)
                }
            }
            Err(error) => Err(error),
        }
    }

    pub fn reset(&self) -> Result<(), ControlError> {
        let snapshot = self.state_reader.snapshot();
        if !snapshot.safety.latched {
            return Err(ControlError::Rejected);
        }

        match self.health.health(&self.motor_id) {
            Some(Health::Ready) => {}
            Some(Health::Degraded(reason)) => {
                return Err(ControlError::DeviceUnavailable(reason));
            }
            Some(Health::Disconnected) | None => {
                return Err(ControlError::DeviceUnavailable(
                    "motor is disconnected".to_owned(),
                ));
            }
        }

        if self.motor.stop()? != MotionState::Stopped {
            return Err(ControlError::Unsafe);
        }

        self.motor.clear_emergency_stop()?;
        if !self
            .safety_writer
            .clear_if_generation(snapshot.safety.generation)
        {
            let _ = self.motor.emergency_stop();
            return Err(ControlError::Unsafe);
        }

        let safe_control = ControlState {
            mode: ControlMode::SafeStop,
            active_lease: None,
        };
        if !self
            .control_writer
            .compare_and_set(safe_control, ControlState::default())
        {
            if self.state_reader.snapshot().safety.latched {
                let _ = self.motor.emergency_stop();
                self.control_writer.force_safe_stop();
                return Err(ControlError::Unsafe);
            }
            return Err(ControlError::StateConflict);
        }

        // A new stop can arrive while the hardware latch is being reset.
        // Reassert the device latch if that happened during this operation.
        if self.state_reader.snapshot().safety.latched {
            let _ = self.motor.emergency_stop();
            self.control_writer.force_safe_stop();
            return Err(ControlError::Unsafe);
        }

        if !self
            .motion_writer
            .report_motion_if_safety_clear(MotionState::Stopped, snapshot.safety.generation)
        {
            let _ = self.motor.emergency_stop();
            self.control_writer.force_safe_stop();
            return Err(ControlError::Unsafe);
        }
        Ok(())
    }
}
