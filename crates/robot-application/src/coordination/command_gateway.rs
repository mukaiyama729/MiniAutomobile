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
        Arc,
        atomic::{AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};
use tokio::sync::{mpsc, oneshot};

static NEXT_LEASE_ID: AtomicU64 = AtomicU64::new(1);
const MAX_CONTROL_LEASE_TTL: Duration = Duration::from_secs(5);
const COMMAND_QUEUE_CAPACITY: usize = 32;

type Reply<T> = oneshot::Sender<Result<T, ControlError>>;

enum GatewayRequest {
    AcquireControl {
        source: CommandSource,
        mode: ControlMode,
        ttl: Duration,
        reply: Reply<ControlLease>,
    },
    RenewControl {
        lease: ControlLease,
        ttl: Duration,
        reply: Reply<ControlLease>,
    },
    ReleaseControl {
        lease: ControlLease,
        reply: Reply<()>,
    },
    Submit {
        proposal: CommandProposal,
        reply: Reply<MotionState>,
    },
    ResetSafetyStop {
        reply: Reply<()>,
    },
    Status {
        reply: Reply<MotionState>,
    },
}

/// Async request facade. A coordinator task owns each accepted ordinary
/// operation through completion, even if its caller drops the response future.
pub struct CommandGateway {
    tx: mpsc::Sender<GatewayRequest>,
    safety: Arc<SafetySupervisor>,
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
    ) -> Self {
        let (tx, rx) = mpsc::channel(COMMAND_QUEUE_CAPACITY);
        let coordinator = CommandCoordinator {
            state_reader,
            control_writer,
            motion_writer,
            motor,
            health,
            motor_id: motor_id.into(),
            arbiter: ResourceArbiter,
            safety: Arc::clone(&safety),
        };
        tokio::spawn(coordinator.run(rx));
        Self { tx, safety }
    }

    pub async fn acquire_control(
        &self,
        source: CommandSource,
        mode: ControlMode,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError> {
        let result = self
            .request(|reply| GatewayRequest::AcquireControl {
                source,
                mode,
                ttl,
                reply,
            })
            .await;
        if needs_fail_safe(&result) {
            let _ = self
                .safety
                .emergency_stop("control coordinator failed during acquisition".to_owned())
                .await;
        }
        result
    }

    pub async fn renew_control(
        &self,
        lease: ControlLease,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError> {
        let result = self
            .request(|reply| GatewayRequest::RenewControl { lease, ttl, reply })
            .await;
        if needs_fail_safe(&result) {
            let _ = self
                .safety
                .emergency_stop("control lease renewal could not be completed".to_owned())
                .await;
        }
        result
    }

    pub async fn release_control(&self, lease: ControlLease) -> Result<(), ControlError> {
        let result = self
            .request(|reply| GatewayRequest::ReleaseControl { lease, reply })
            .await;
        if needs_fail_safe(&result) {
            let _ = self
                .safety
                .emergency_stop("control release could not be completed".to_owned())
                .await;
        }
        result
    }

    pub async fn submit(&self, proposal: CommandProposal) -> Result<MotionState, ControlError> {
        let result = self
            .request(|reply| GatewayRequest::Submit { proposal, reply })
            .await;
        if needs_fail_safe(&result) {
            let _ = self
                .safety
                .emergency_stop("motion command could not be completed".to_owned())
                .await;
        }
        result
    }

    /// Emergency stop deliberately bypasses the ordinary command queue.
    pub async fn emergency_stop(&self, reason: String) -> Result<MotionState, ControlError> {
        self.safety.emergency_stop(reason).await
    }

    pub async fn reset_safety_stop(&self) -> Result<(), ControlError> {
        self.request(|reply| GatewayRequest::ResetSafetyStop { reply })
            .await
    }

    pub async fn status(&self) -> Result<MotionState, ControlError> {
        self.request(|reply| GatewayRequest::Status { reply }).await
    }

    async fn request<T>(
        &self,
        make_request: impl FnOnce(Reply<T>) -> GatewayRequest,
    ) -> Result<T, ControlError> {
        let (reply, response) = oneshot::channel();
        match self.tx.try_send(make_request(reply)) {
            Ok(()) => {}
            Err(mpsc::error::TrySendError::Full(_)) => {
                return Err(ControlError::CommandQueueFull);
            }
            Err(mpsc::error::TrySendError::Closed(_)) => {
                return Err(ControlError::CoordinatorUnavailable);
            }
        }
        response
            .await
            .map_err(|_| ControlError::CoordinatorUnavailable)?
    }
}

fn needs_fail_safe<T>(result: &Result<T, ControlError>) -> bool {
    matches!(
        result,
        Err(ControlError::CommandQueueFull | ControlError::CoordinatorUnavailable)
    )
}

struct CommandCoordinator {
    state_reader: Arc<dyn RobotStateReader>,
    control_writer: Arc<dyn ControlStateWriter>,
    motion_writer: Arc<dyn RobotMotionStateWriter>,
    motor: Arc<dyn MotorPort>,
    health: Arc<dyn DeviceHealthReader>,
    motor_id: String,
    arbiter: ResourceArbiter,
    safety: Arc<SafetySupervisor>,
}

impl CommandCoordinator {
    async fn run(self, mut rx: mpsc::Receiver<GatewayRequest>) {
        while let Some(request) = rx.recv().await {
            match request {
                GatewayRequest::AcquireControl {
                    source,
                    mode,
                    ttl,
                    reply,
                } => {
                    let result = self.acquire_control(source, mode, ttl).await;
                    if let Err(Ok(lease)) = reply.send(result) {
                        let _ = self.release_control(lease).await;
                    }
                }
                GatewayRequest::RenewControl { lease, ttl, reply } => {
                    let result = self.renew_control(lease, ttl).await;
                    if let Err(Ok(renewed)) = reply.send(result) {
                        let _ = self.release_control(renewed).await;
                    }
                }
                GatewayRequest::ReleaseControl { lease, reply } => {
                    let _ = reply.send(self.release_control(lease).await);
                }
                GatewayRequest::Submit { proposal, reply } => {
                    let _ = reply.send(self.submit(proposal).await);
                }
                GatewayRequest::ResetSafetyStop { reply } => {
                    let _ = reply.send(self.safety.reset().await);
                }
                GatewayRequest::Status { reply } => {
                    let _ = reply.send(self.status().await);
                }
            }
        }
    }

    async fn acquire_control(
        &self,
        source: CommandSource,
        mode: ControlMode,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError> {
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
                return Err(ControlError::ResourceBusy);
            }

            // An expired owner's output must be stopped before another owner
            // can take the resource, even if the adapter's deadline is racing.
            let stopped = match self.motor.stop().await {
                Ok(MotionState::Stopped) => MotionState::Stopped,
                Ok(state) => {
                    let _ = self
                        .safety
                        .emergency_stop(format!(
                            "device did not stop after control lease expiration: {state:?}"
                        ))
                        .await;
                    return Err(ControlError::Unsafe);
                }
                Err(error) => {
                    let _ = self
                        .safety
                        .emergency_stop(format!("could not stop expired control owner: {error}"))
                        .await;
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

        // Start the new lease duration only after the stop acknowledgement.
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

    async fn renew_control(
        &self,
        lease: ControlLease,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError> {
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
                .emergency_stop("control lease expired before renewal".to_owned())
                .await;
            return Err(ControlError::Expired);
        }
        self.renew_locked(
            snapshot.control,
            active,
            expires_at,
            snapshot.safety.generation,
        )
        .await
    }

    async fn renew_locked(
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
        if let Err(error) = self.motor.renew_lease(lease.id, lease.expires_at).await {
            let _ = self.control_writer.compare_and_set(next, current);
            let _ = self
                .safety
                .emergency_stop(format!("control lease renewal failed: {error}"))
                .await;
            return Err(error);
        }
        Ok(lease)
    }

    async fn release_control(&self, lease: ControlLease) -> Result<(), ControlError> {
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

        match self.motor.stop().await {
            Ok(MotionState::Stopped) => {
                if !self
                    .motion_writer
                    .report_motion_if_safety_clear(MotionState::Stopped, snapshot.safety.generation)
                {
                    return Err(ControlError::Unsafe);
                }
            }
            Ok(state) => {
                let _ = self
                    .safety
                    .emergency_stop(format!(
                        "device did not stop while releasing control: {state:?}"
                    ))
                    .await;
                return Err(ControlError::Unsafe);
            }
            Err(error) => {
                let _ = self
                    .safety
                    .emergency_stop(format!("failed to release control safely: {error}"))
                    .await;
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

    async fn submit(&self, proposal: CommandProposal) -> Result<MotionState, ControlError> {
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
            .await
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
                    .emergency_stop(format!("motion output failed: {error}"))
                    .await;
                Err(error)
            }
        }
    }

    async fn status(&self) -> Result<MotionState, ControlError> {
        let snapshot = self.state_reader.snapshot();
        if snapshot.safety.latched {
            return Err(ControlError::Unsafe);
        }
        let state = match self.motor.read_state().await {
            Ok(state) => state,
            Err(error) => {
                let _ = self
                    .safety
                    .emergency_stop(format!("motor status read failed: {error}"))
                    .await;
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
