use robot_domain::{CommandProposal, RejectionReason, StateSnapshot};
use std::time::Instant;

/// Applies control ownership, mode, safety, resource, and freshness rules
/// before a proposal can reach a device port.
#[derive(Default)]
pub struct ResourceArbiter;

impl ResourceArbiter {
    pub fn authorize(
        &self,
        state: &StateSnapshot,
        proposal: &CommandProposal,
        now: Instant,
    ) -> Result<(), RejectionReason> {
        if state.safety.latched || state.control.mode == robot_domain::ControlMode::SafeStop {
            return Err(RejectionReason::Unsafe);
        }
        if proposal.issued_at > now || proposal.valid_until <= now {
            return Err(RejectionReason::Expired);
        }
        if !state.control.mode.accepts(proposal.source) {
            return Err(RejectionReason::InvalidMode);
        }

        let lease = state
            .control
            .active_lease
            .ok_or(RejectionReason::NoAuthority)?;
        if lease.expires_at <= now || proposal.valid_until != lease.expires_at {
            return Err(RejectionReason::Expired);
        }
        if lease.owner != proposal.source || proposal.control_lease_id != Some(lease.id) {
            return Err(RejectionReason::NoAuthority);
        }

        match proposal.resource {
            robot_domain::ResourceId::DriveBase => Ok(()),
        }
    }
}
