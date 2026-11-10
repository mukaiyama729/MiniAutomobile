use super::{ResourceArbiter, SafetySupervisor};
use crate::ports::outbound::{
    ControlError, ControlStateWriter, DeviceHealthReader, MotorPort, RobotMotionStateWriter,
    RobotStateReader,
};
use robot_domain::{
    CommandProposal, CommandSource, ControlLease, ControlMode, ControlState, Health, MotionCommand,
    MotionState, RejectionReason,
};
use std::{
    sync::{
        Arc, Mutex, MutexGuard,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};

static NEXT_LEASE_ID: AtomicU64 = AtomicU64::new(1);
const MAX_CONTROL_LEASE_TTL: Duration = Duration::from_secs(5);

/// Serializes ordinary control changes and commands. The emergency-stop path
/// deliberately does not acquire this lock.
pub struct CommandGateway {
    state_reader: Arc<dyn RobotStateReader>,
    control_writer: Arc<dyn ControlStateWriter>,
    motion_writer: Arc<dyn RobotMotionStateWriter>,
    motor: Arc<dyn MotorPort>,
    health: Arc<dyn DeviceHealthReader>,
    motor_id: String,
    arbiter: ResourceArbiter,
    safety: Arc<SafetySupervisor>,
    control_gate: Arc<Mutex<()>>,
}

impl CommandGateway {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        state_reader: Arc<dyn RobotStateReader>,
        control_writer: Arc<dyn ControlStateWriter>,
        motion_writer: Arc<dyn RobotMotionStateWriter>,
        motor: Arc<dyn MotorPort>,
        health: Arc<dyn DeviceHealthReader>,
        motor_id: impl Into<String>,
        safety: Arc<SafetySupervisor>,
        control_gate: Arc<Mutex<()>>,
    ) -> Self {
        Self {
            state_reader,
            control_writer,
            motion_writer,
            motor,
            health,
            motor_id: motor_id.into(),
            arbiter: ResourceArbiter,
            safety,
            control_gate,
        }
    }

    pub fn acquire_control(
        &self,
        source: CommandSource,
        mode: ControlMode,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError> {
        let _guard = self.lock()?;
        let now = Instant::now();
        let snapshot = self.state_reader.snapshot();
        if snapshot.safety.latched
            || snapshot.control.mode == ControlMode::SafeStop
            || mode == ControlMode::SafeStop
        {
            return Err(ControlError::Unsafe);
        }
        if !mode.accepts(source) {
            return Err(ControlError::InvalidMode);
        }

        if let Some(current) = snapshot.control.active_lease {
            if current.expires_at > now {
                // Possession of the current lease is required to continue
                // control. Matching a logical source is not enough to let a
                // second adapter instance take over or renew it implicitly.
                return Err(ControlError::ResourceBusy);
            }

            // An expired owner's output must be stopped before another owner
            // can take the resource, even if the adapter's deadline is racing.
            let stopped = match self.motor.stop() {
                Ok(MotionState::Stopped) => MotionState::Stopped,
                Ok(state) => {
                    let _ = self.safety.emergency_stop(format!(
                        "device did not stop after control lease expiration: {state:?}"
                    ));
                    return Err(ControlError::Unsafe);
                }
                Err(error) => {
                    let _ = self
                        .safety
                        .emergency_stop(format!("could not stop expired control owner: {error}"));
                    return Err(error);
                }
            };
            if !self
                .motion_writer
                .report_motion_if_safety_clear(stopped, snapshot.safety.generation)
            {
                return Err(ControlError::Unsafe);
            }
        }

        // Stopping an expired owner can take up to the device ACK timeout.
        // Start the new lease duration only after that acknowledgement.
        let expires_at = deadline(Instant::now(), ttl)?;
        let lease = ControlLease {
            id: NEXT_LEASE_ID.fetch_add(1, Ordering::Relaxed),
            owner: source,
            expires_at,
        };
        let next = ControlState {
            mode,
            active_lease: Some(lease),
        };
        if !self.control_writer.compare_and_set_if_safety_clear(
            snapshot.control,
            next,
            snapshot.safety.generation,
        ) {
            return Err(ControlError::StateConflict);
        }
        Ok(lease)
    }

    pub fn renew_control(
        &self,
        lease: ControlLease,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError> {
        let _guard = self.lock()?;
        let snapshot = self.state_reader.snapshot();
        let now = Instant::now();
        let expires_at = deadline(now, ttl)?;
        if snapshot.safety.latched || snapshot.control.mode == ControlMode::SafeStop {
            return Err(ControlError::Unsafe);
        }
        let active = snapshot
            .control
            .active_lease
            .ok_or(ControlError::InvalidLease)?;
        if active.id != lease.id
            || active.owner != lease.owner
            || active.expires_at != lease.expires_at
        {
            return Err(ControlError::InvalidLease);
        }
        if now >= active.expires_at {
            let _ = self
                .safety
                .emergency_stop("control lease expired before renewal".to_owned());
            return Err(ControlError::Expired);
        }
        self.renew_locked(
            snapshot.control,
            active,
            expires_at,
            snapshot.safety.generation,
        )
    }

    fn renew_locked(
        &self,
        current: ControlState,
        active: ControlLease,
        expires_at: Instant,
        safety_generation: u64,
    ) -> Result<ControlLease, ControlError> {
        if expires_at <= active.expires_at {
            return Err(ControlError::LeaseNotExtended);
        }
        let lease = ControlLease {
            expires_at,
            ..active
        };
        let next = ControlState {
            active_lease: Some(lease),
            ..current
        };
        if !self
            .control_writer
            .compare_and_set_if_safety_clear(current, next, safety_generation)
        {
            return Err(ControlError::StateConflict);
        }
        if let Err(error) = self.motor.renew_lease(lease.id, lease.expires_at) {
            let _ = self.control_writer.compare_and_set(next, current);
            let _ = self
                .safety
                .emergency_stop(format!("control lease renewal failed: {error}"));
            return Err(error);
        }
        Ok(lease)
    }

    pub fn release_control(&self, lease: ControlLease) -> Result<(), ControlError> {
        let _guard = self.lock()?;
        let snapshot = self.state_reader.snapshot();
        let active = snapshot
            .control
            .active_lease
            .ok_or(ControlError::InvalidLease)?;
        if active.id != lease.id
            || active.owner != lease.owner
            || active.expires_at != lease.expires_at
        {
            return Err(ControlError::InvalidLease);
        }

        match self.motor.stop() {
            Ok(MotionState::Stopped) => {
                if !self
                    .motion_writer
                    .report_motion_if_safety_clear(MotionState::Stopped, snapshot.safety.generation)
                {
                    return Err(ControlError::Unsafe);
                }
            }
            Ok(state) => {
                let _ = self.safety.emergency_stop(format!(
                    "device did not stop while releasing control: {state:?}"
                ));
                return Err(ControlError::Unsafe);
            }
            Err(error) => {
                let _ = self
                    .safety
                    .emergency_stop(format!("failed to release control safely: {error}"));
                return Err(error);
            }
        }

        if !self
            .control_writer
            .compare_and_set(snapshot.control, ControlState::default())
        {
            return Err(ControlError::StateConflict);
        }
        Ok(())
    }

    pub fn submit(&self, proposal: CommandProposal) -> Result<MotionState, ControlError> {
        let _guard = self.lock()?;
        let snapshot = self.state_reader.snapshot();
        let now = Instant::now();
        self.arbiter
            .authorize(&snapshot, &proposal, now)
            .map_err(map_rejection)?;

        match self.health.health(&self.motor_id) {
            Some(Health::Ready) => {}
            Some(Health::Degraded(_)) if proposal.command == MotionCommand::Stop => {}
            Some(Health::Degraded(reason)) => {
                return Err(ControlError::DeviceUnavailable(reason));
            }
            Some(Health::Disconnected) | None => {
                if proposal.command != MotionCommand::Stop {
                    return Err(ControlError::DeviceUnavailable(
                        "motor is disconnected".to_owned(),
                    ));
                }
            }
        }

        let lease_id = proposal.control_lease_id.ok_or(ControlError::NoAuthority)?;
        match self
            .motor
            .command(proposal.command, lease_id, proposal.valid_until)
        {
            Ok(state) => {
                if self
                    .motion_writer
                    .report_motion_if_safety_clear(state, snapshot.safety.generation)
                {
                    Ok(state)
                } else {
                    Err(ControlError::Unsafe)
                }
            }
            Err(error) => {
                let _ = self
                    .safety
                    .emergency_stop(format!("motion output failed: {error}"));
                Err(error)
            }
        }
    }

    pub fn emergency_stop(&self, reason: String) -> Result<MotionState, ControlError> {
        self.safety.emergency_stop(reason)
    }

    pub fn reset_safety_stop(&self) -> Result<(), ControlError> {
        let _guard = self.lock()?;
        self.safety.reset()
    }

    pub fn status(&self) -> Result<MotionState, ControlError> {
        let _guard = self.lock()?;
        let snapshot = self.state_reader.snapshot();
        if snapshot.safety.latched {
            return Err(ControlError::Unsafe);
        }
        let state = match self.motor.read_state() {
            Ok(state) => state,
            Err(error) => {
                let _ = self
                    .safety
                    .emergency_stop(format!("motor status read failed: {error}"));
                return Err(error);
            }
        };
        if !self
            .motion_writer
            .report_motion_if_safety_clear(state, snapshot.safety.generation)
        {
            return Err(ControlError::Unsafe);
        }
        Ok(state)
    }

    fn lock(&self) -> Result<MutexGuard<'_, ()>, ControlError> {
        self.control_gate
            .lock()
            .map_err(|_| ControlError::StateConflict)
    }
}

fn deadline(now: Instant, ttl: Duration) -> Result<Instant, ControlError> {
    if ttl.is_zero() || ttl > MAX_CONTROL_LEASE_TTL {
        return Err(ControlError::InvalidDuration);
    }
    now.checked_add(ttl).ok_or(ControlError::InvalidDuration)
}

fn map_rejection(reason: RejectionReason) -> ControlError {
    match reason {
        RejectionReason::NoAuthority => ControlError::NoAuthority,
        RejectionReason::Expired => ControlError::Expired,
        RejectionReason::ResourceBusy => ControlError::ResourceBusy,
        RejectionReason::Unsafe => ControlError::Unsafe,
        RejectionReason::InvalidMode => ControlError::InvalidMode,
    }
}
