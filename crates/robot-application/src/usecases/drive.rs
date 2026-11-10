use crate::{
    coordination::CommandGateway,
    ports::{inbound::DriveControl, outbound::ControlError},
};
use async_trait::async_trait;
use robot_domain::{
    CommandProposal, CommandSource, ControlLease, ControlMode, Direction, MotionCommand,
    MotionState, ResourceId,
};
use std::{sync::Arc, time::Duration};

/// Input-facing drive use case. All ordinary motion requests pass through the
/// command gateway; callers never receive the hardware port.
pub struct DriveUseCase {
    gateway: Arc<CommandGateway>,
    source: CommandSource,
    mode: ControlMode,
}

impl DriveUseCase {
    /// Bind one inbound adapter to its trusted logical source and mode at the
    /// composition root. Create a separate instance for each actor boundary.
    pub fn new(gateway: Arc<CommandGateway>, source: CommandSource, mode: ControlMode) -> Self {
        Self {
            gateway,
            source,
            mode,
        }
    }

    async fn propose(
        &self,
        lease: ControlLease,
        command: MotionCommand,
    ) -> Result<MotionState, ControlError> {
        if lease.owner != self.source {
            return Err(ControlError::InvalidLease);
        }
        let proposal = CommandProposal {
            source: self.source,
            resource: ResourceId::DriveBase,
            command,
            issued_at: std::time::Instant::now(),
            valid_until: lease.expires_at,
            control_lease_id: Some(lease.id),
        };
        self.gateway.submit(proposal).await
    }
}

#[async_trait]
impl DriveControl for DriveUseCase {
    async fn acquire_control(&self, ttl: Duration) -> Result<ControlLease, ControlError> {
        self.gateway
            .acquire_control(self.source, self.mode, ttl)
            .await
    }

    async fn renew_control(
        &self,
        lease: ControlLease,
        ttl: Duration,
    ) -> Result<ControlLease, ControlError> {
        if lease.owner != self.source {
            return Err(ControlError::InvalidLease);
        }
        self.gateway.renew_control(lease, ttl).await
    }

    async fn release_control(&self, lease: ControlLease) -> Result<(), ControlError> {
        if lease.owner != self.source {
            return Err(ControlError::InvalidLease);
        }
        self.gateway.release_control(lease).await
    }

    async fn drive(
        &self,
        lease: ControlLease,
        direction: Direction,
    ) -> Result<MotionState, ControlError> {
        self.propose(lease, MotionCommand::SetDirection(direction))
            .await
    }

    async fn stop(&self, lease: ControlLease) -> Result<MotionState, ControlError> {
        self.propose(lease, MotionCommand::Stop).await
    }

    async fn emergency_stop(&self, reason: String) -> Result<MotionState, ControlError> {
        self.gateway.emergency_stop(reason).await
    }

    async fn reset_safety_stop(&self) -> Result<(), ControlError> {
        self.gateway.reset_safety_stop().await
    }

    async fn status(&self) -> Result<MotionState, ControlError> {
        self.gateway.status().await
    }
}
