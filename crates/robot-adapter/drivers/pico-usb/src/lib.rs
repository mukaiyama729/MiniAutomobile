//! USB CDC driver for one Pico 2.
//!
//! The serial worker is the only owner of the port. It refreshes the watchdog
//! only while an application control lease is valid, re-sending motion while
//! moving and PINGing while stopped. On expiry it sends STOP and ceases
//! background watchdog traffic.

use async_trait::async_trait;
use robot_application::ports::outbound::{ControlError, MotorPort};
use robot_device_manager::Device;
use robot_domain::{Capability, Health, MotionCommand, MotionState};
use robot_protocol::{self as protocol, Command, Request, Response};
use std::{
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, AtomicU64, Ordering},
    },
    time::{Duration, Instant},
};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    sync::{Notify, mpsc, oneshot},
    time::timeout,
};
use tokio_serial::{SerialPortBuilderExt, SerialStream};

const HEARTBEAT: Duration = Duration::from_millis(300);
const ACK_TIMEOUT: Duration = Duration::from_millis(650);
const WRITE_TIMEOUT: Duration = Duration::from_millis(50);
const CALL_TIMEOUT: Duration = Duration::from_secs(2);
const CAPABILITIES: &[Capability] = &[Capability::Motor];

type MotionReply = oneshot::Sender<Result<MotionState, ControlError>>;
type UnitReply = oneshot::Sender<Result<(), ControlError>>;

struct Cancellation {
    cancelled: AtomicBool,
    notify: Notify,
}

impl Cancellation {
    fn new() -> Self {
        Self {
            cancelled: AtomicBool::new(false),
            notify: Notify::new(),
        }
    }

    fn is_cancelled(&self) -> bool {
        self.cancelled.load(Ordering::Acquire)
    }

    fn cancel(&self) {
        self.cancelled.store(true, Ordering::Release);
        self.notify.notify_one();
    }

    async fn cancelled(&self) {
        loop {
            if self.is_cancelled() {
                return;
            }
            self.notify.notified().await;
        }
    }
}

struct CancelOnDrop(Option<Arc<Cancellation>>);

impl Drop for CancelOnDrop {
    fn drop(&mut self) {
        if let Some(cancelled) = &self.0 {
            cancelled.cancel();
        }
    }
}

enum Job {
    Command {
        command: Command,
        lease_id: u64,
        valid_until: Instant,
        cancelled: Arc<Cancellation>,
        reply: MotionReply,
    },
    RenewLease {
        lease_id: u64,
        valid_until: Instant,
        cancelled: Arc<Cancellation>,
        reply: UnitReply,
    },
    Stop {
        reply: MotionReply,
    },
    EmergencyStop {
        reply: MotionReply,
    },
    ClearEmergencyStop {
        generation: u64,
        cancelled: Arc<Cancellation>,
        reply: UnitReply,
    },
    ReadState {
        reply: MotionReply,
    },
}

pub struct PicoUsbMotor {
    id: String,
    tx: mpsc::UnboundedSender<Job>,
    health: Arc<Mutex<Health>>,
    stop_generation: Arc<AtomicU64>,
    stop_notify: Arc<Notify>,
}

impl PicoUsbMotor {
    pub async fn connect(id: impl Into<String>, path: &str) -> Result<Self, ControlError> {
        let id = id.into();
        let path = path.to_owned();
        // Port opening is synchronous in tokio-serial; keep it off the runtime
        // worker while all serial reads and writes remain async.
        let mut port = tokio::task::spawn_blocking(move || {
            tokio_serial::new(path, 115_200).open_native_async()
        })
        .await
        .map_err(|error| ControlError::Transport(error.to_string()))?
        .map_err(|error| ControlError::Transport(error.to_string()))?;

        let stop_generation = Arc::new(AtomicU64::new(0));
        let stop_notify = Arc::new(Notify::new());
        let mut sequence = 1;
        // Establish that the controller is responding and start from a known
        // stopped state before exposing the adapter to application code.
        let initial_state = exchange_next(
            &mut port,
            &mut sequence,
            Command::Stop,
            &stop_generation,
            &stop_notify,
            None,
            None,
        )
        .await?;
        if initial_state != MotionState::Stopped {
            return Err(ControlError::Rejected);
        }

        let (tx, rx) = mpsc::unbounded_channel();
        let health = Arc::new(Mutex::new(Health::Ready));
        let worker_health = Arc::clone(&health);
        let worker_stop_generation = Arc::clone(&stop_generation);
        let worker_stop_notify = Arc::clone(&stop_notify);
        tokio::spawn(serial_worker(
            port,
            rx,
            worker_health,
            worker_stop_generation,
            worker_stop_notify,
            sequence,
        ));

        Ok(Self {
            id,
            tx,
            health,
            stop_generation,
            stop_notify,
        })
    }

    async fn wait_motion(
        &self,
        receiver: oneshot::Receiver<Result<MotionState, ControlError>>,
        cancelled: Option<Arc<Cancellation>>,
    ) -> Result<MotionState, ControlError> {
        let _cancel_on_drop = CancelOnDrop(cancelled);
        match timeout(CALL_TIMEOUT, receiver).await {
            Ok(Ok(result)) => result,
            Ok(Err(_)) => Err(ControlError::Disconnected),
            Err(_) => Err(ControlError::Timeout),
        }
    }

    async fn wait_unit(
        &self,
        receiver: oneshot::Receiver<Result<(), ControlError>>,
        cancelled: Option<Arc<Cancellation>>,
    ) -> Result<(), ControlError> {
        let _cancel_on_drop = CancelOnDrop(cancelled);
        match timeout(CALL_TIMEOUT, receiver).await {
            Ok(Ok(result)) => result,
            Ok(Err(_)) => Err(ControlError::Disconnected),
            Err(_) => Err(ControlError::Timeout),
        }
    }

    async fn request_motion(
        &self,
        command: Command,
        lease_id: u64,
        valid_until: Instant,
    ) -> Result<MotionState, ControlError> {
        if is_latched(&self.stop_generation) && command != Command::Stop {
            return Err(ControlError::Unsafe);
        }
        let cancelled = Arc::new(Cancellation::new());
        let (reply, receiver) = oneshot::channel();
        self.tx
            .send(Job::Command {
                command,
                lease_id,
                valid_until,
                cancelled: Arc::clone(&cancelled),
                reply,
            })
            .map_err(|_| ControlError::Disconnected)?;
        self.wait_motion(receiver, Some(cancelled)).await
    }

    async fn request_stop(&self) -> Result<MotionState, ControlError> {
        let (reply, receiver) = oneshot::channel();
        self.tx
            .send(Job::Stop { reply })
            .map_err(|_| ControlError::Disconnected)?;
        self.wait_motion(receiver, None).await
    }
}

#[async_trait]
impl MotorPort for PicoUsbMotor {
    async fn command(
        &self,
        command: MotionCommand,
        lease_id: u64,
        valid_until: Instant,
    ) -> Result<MotionState, ControlError> {
        let wire_command = match command {
            MotionCommand::SetDirection(robot_domain::Direction::Forward) => Command::Forward,
            MotionCommand::SetDirection(robot_domain::Direction::Backward) => Command::Backward,
            MotionCommand::Stop => Command::Stop,
        };
        self.request_motion(wire_command, lease_id, valid_until)
            .await
    }

    async fn renew_lease(&self, lease_id: u64, valid_until: Instant) -> Result<(), ControlError> {
        if is_latched(&self.stop_generation) {
            return Err(ControlError::Unsafe);
        }
        let cancelled = Arc::new(Cancellation::new());
        let (reply, receiver) = oneshot::channel();
        self.tx
            .send(Job::RenewLease {
                lease_id,
                valid_until,
                cancelled: Arc::clone(&cancelled),
                reply,
            })
            .map_err(|_| ControlError::Disconnected)?;
        self.wait_unit(receiver, Some(cancelled)).await
    }

    async fn stop(&self) -> Result<MotionState, ControlError> {
        self.request_stop().await
    }

    async fn emergency_stop(&self) -> Result<MotionState, ControlError> {
        latch_stop(&self.stop_generation, &self.stop_notify);
        let (reply, receiver) = oneshot::channel();
        self.tx
            .send(Job::EmergencyStop { reply })
            .map_err(|_| ControlError::Disconnected)?;
        self.wait_motion(receiver, None).await
    }

    async fn clear_emergency_stop(&self) -> Result<(), ControlError> {
        let generation = self.stop_generation.load(Ordering::Acquire);
        if generation & 1 == 0 {
            return Err(ControlError::Rejected);
        }
        let cancelled = Arc::new(Cancellation::new());
        let (reply, receiver) = oneshot::channel();
        self.tx
            .send(Job::ClearEmergencyStop {
                generation,
                cancelled: Arc::clone(&cancelled),
                reply,
            })
            .map_err(|_| ControlError::Disconnected)?;
        self.wait_unit(receiver, Some(cancelled)).await?;
        if self.stop_generation.load(Ordering::Acquire) != generation.wrapping_add(1) {
            return Err(ControlError::Unsafe);
        }
        Ok(())
    }

    async fn read_state(&self) -> Result<MotionState, ControlError> {
        let (reply, receiver) = oneshot::channel();
        self.tx
            .send(Job::ReadState { reply })
            .map_err(|_| ControlError::Disconnected)?;
        self.wait_motion(receiver, None).await
    }
}

impl Device for PicoUsbMotor {
    fn id(&self) -> &str {
        &self.id
    }

    fn capabilities(&self) -> &'static [Capability] {
        CAPABILITIES
    }

    fn health(&self) -> Health {
        self.health
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .clone()
    }
}

#[derive(Clone, Copy)]
struct ActiveLease {
    id: u64,
    expires_at: Instant,
    motion: Option<Command>,
}

async fn serial_worker(
    mut port: SerialStream,
    mut rx: mpsc::UnboundedReceiver<Job>,
    health: Arc<Mutex<Health>>,
    stop_generation: Arc<AtomicU64>,
    stop_notify: Arc<Notify>,
    mut sequence: u16,
) {
    let mut active_lease: Option<ActiveLease> = None;
    let mut last_lease_id = None;
    let mut last_heartbeat = Instant::now();
    let mut handled_stop_generation = None;
    let mut confirmed_stop_generation = None;
    let mut last_stop_attempt = Instant::now();
    let mut last_stop_result = Ok(MotionState::Stopped);
    let mut last_state = MotionState::Stopped;

    loop {
        let now = Instant::now();
        let stop_value = stop_generation.load(Ordering::Acquire);
        if stop_value & 1 == 1 {
            active_lease = None;
            if handled_stop_generation != Some(stop_value)
                || last_stop_attempt.elapsed() >= HEARTBEAT
            {
                last_stop_result = send_stop(
                    &mut port,
                    &mut sequence,
                    &stop_generation,
                    &stop_notify,
                    &health,
                )
                .await;
                if let Ok(state) = last_stop_result {
                    last_state = state;
                    if state == MotionState::Stopped {
                        confirmed_stop_generation = Some(stop_value);
                    } else {
                        confirmed_stop_generation = None;
                    }
                } else {
                    confirmed_stop_generation = None;
                }
                handled_stop_generation = Some(stop_value);
                last_stop_attempt = Instant::now();
            }
        } else if let Some(active) = active_lease {
            if now >= active.expires_at {
                active_lease = None;
                last_stop_result = send_stop(
                    &mut port,
                    &mut sequence,
                    &stop_generation,
                    &stop_notify,
                    &health,
                )
                .await;
                if let Ok(state) = last_stop_result {
                    last_state = state;
                }
                last_heartbeat = Instant::now();
            } else if last_heartbeat.elapsed() >= HEARTBEAT {
                // Refresh a moving command itself so PING and STATUS are not
                // required to keep the firmware's motion watchdog alive.
                let heartbeat = active.motion.unwrap_or(Command::Ping);
                let result = exchange_next(
                    &mut port,
                    &mut sequence,
                    heartbeat,
                    &stop_generation,
                    &stop_notify,
                    None,
                    Some(active.expires_at),
                )
                .await;
                last_heartbeat = Instant::now();
                update_health(&health, &result);
                match result {
                    Ok(state) => {
                        last_state = state;
                        if state == MotionState::Stopped {
                            if let Some(current) = active_lease.as_mut() {
                                current.motion = None;
                            }
                        }
                    }
                    Err(_) => {
                        active_lease = None;
                        last_stop_result = send_stop(
                            &mut port,
                            &mut sequence,
                            &stop_generation,
                            &stop_notify,
                            &health,
                        )
                        .await;
                        if let Ok(state) = last_stop_result {
                            last_state = state;
                        }
                    }
                }
            }
        }

        let now = Instant::now();
        let timeout = if stop_generation.load(Ordering::Acquire) & 1 == 1 {
            Some(HEARTBEAT.saturating_sub(last_stop_attempt.elapsed()))
        } else if let Some(active) = active_lease {
            Some(
                active
                    .expires_at
                    .saturating_duration_since(now)
                    .min(HEARTBEAT.saturating_sub(last_heartbeat.elapsed())),
            )
        } else {
            None
        };

        let job = tokio::select! {
            biased;
            job = rx.recv() => job,
            _ = tokio::time::sleep(timeout.unwrap_or_default()), if timeout.is_some() => continue,
        };
        let Some(job) = job else {
            let _ = send_stop(
                &mut port,
                &mut sequence,
                &stop_generation,
                &stop_notify,
                &health,
            )
            .await;
            set_health(&health, Health::Disconnected);
            break;
        };

        match job {
            Job::Command {
                command,
                lease_id,
                valid_until,
                cancelled,
                reply,
            } => {
                if cancelled.is_cancelled() {
                    let _ = reply.send(Err(ControlError::Timeout));
                    continue;
                }
                let now = Instant::now();
                if is_latched(&stop_generation) && command != Command::Stop {
                    let _ = reply.send(Err(ControlError::Unsafe));
                    continue;
                }
                if valid_until <= now {
                    let _ = reply.send(Err(ControlError::Expired));
                    continue;
                }
                match active_lease {
                    Some(active) if active.id != lease_id => {
                        let _ = reply.send(Err(ControlError::InvalidLease));
                        continue;
                    }
                    Some(active) if active.expires_at <= now => {
                        active_lease = None;
                        let _ = reply.send(Err(ControlError::Expired));
                        continue;
                    }
                    Some(active) if valid_until != active.expires_at => {
                        let _ = reply.send(Err(ControlError::Expired));
                        continue;
                    }
                    None if last_lease_id == Some(lease_id) => {
                        let _ = reply.send(Err(ControlError::Expired));
                        continue;
                    }
                    None => {
                        last_lease_id = Some(lease_id);
                        active_lease = Some(ActiveLease {
                            id: lease_id,
                            expires_at: valid_until,
                            motion: None,
                        });
                    }
                    _ => {}
                }
                if cancelled.is_cancelled() {
                    let _ = reply.send(Err(ControlError::Timeout));
                    continue;
                }
                if is_latched(&stop_generation) && command != Command::Stop {
                    active_lease = None;
                    let _ = reply.send(Err(ControlError::Unsafe));
                    continue;
                }

                let result = exchange_next(
                    &mut port,
                    &mut sequence,
                    command,
                    &stop_generation,
                    &stop_notify,
                    Some(&cancelled),
                    Some(valid_until),
                )
                .await;
                match &result {
                    Ok(state) if matches!(command, Command::Forward | Command::Backward) => {
                        if valid_until > Instant::now() && !is_latched(&stop_generation) {
                            last_lease_id = Some(lease_id);
                            active_lease = Some(ActiveLease {
                                id: lease_id,
                                expires_at: valid_until,
                                motion: (*state != MotionState::Stopped).then_some(command),
                            });
                            last_heartbeat = Instant::now();
                        } else {
                            active_lease = None;
                        }
                    }
                    Ok(_) if command == Command::Stop => {
                        last_lease_id = Some(lease_id);
                        active_lease = Some(ActiveLease {
                            id: lease_id,
                            expires_at: valid_until,
                            motion: None,
                        });
                    }
                    Err(_) if matches!(command, Command::Forward | Command::Backward) => {
                        active_lease = None;
                        last_stop_result = send_stop(
                            &mut port,
                            &mut sequence,
                            &stop_generation,
                            &stop_notify,
                            &health,
                        )
                        .await;
                        if let Ok(state) = last_stop_result {
                            last_state = state;
                        }
                    }
                    Err(_) if command == Command::Stop => {
                        active_lease = None;
                        last_stop_result = send_stop(
                            &mut port,
                            &mut sequence,
                            &stop_generation,
                            &stop_notify,
                            &health,
                        )
                        .await;
                        if let Ok(state) = last_stop_result {
                            last_state = state;
                        }
                    }
                    _ => {}
                }
                if let Ok(state) = result {
                    last_state = state;
                }
                update_health(&health, &result);
                let _ = reply.send(result);
            }
            Job::RenewLease {
                lease_id,
                valid_until,
                cancelled,
                reply,
            } => {
                let now = Instant::now();
                let result = if cancelled.is_cancelled() {
                    Err(ControlError::Timeout)
                } else if is_latched(&stop_generation) {
                    Err(ControlError::Unsafe)
                } else if valid_until <= now {
                    Err(ControlError::Expired)
                } else {
                    match active_lease.as_mut() {
                        Some(active)
                            if active.id == lease_id
                                && active.expires_at > now
                                && valid_until <= active.expires_at =>
                        {
                            Err(ControlError::LeaseNotExtended)
                        }
                        Some(active) if active.id == lease_id && active.expires_at > now => {
                            active.expires_at = valid_until;
                            Ok(())
                        }
                        Some(active) if active.id == lease_id => Err(ControlError::Expired),
                        None if last_lease_id == Some(lease_id) => Err(ControlError::Expired),
                        None => {
                            last_lease_id = Some(lease_id);
                            active_lease = Some(ActiveLease {
                                id: lease_id,
                                expires_at: valid_until,
                                motion: None,
                            });
                            last_heartbeat = now;
                            Ok(())
                        }
                        Some(_) => Err(ControlError::InvalidLease),
                    }
                };
                let _ = reply.send(result);
            }
            Job::Stop { reply } => {
                active_lease = None;
                let result = exchange_next(
                    &mut port,
                    &mut sequence,
                    Command::Stop,
                    &stop_generation,
                    &stop_notify,
                    None,
                    None,
                )
                .await;
                if let Ok(state) = result {
                    last_state = state;
                    last_stop_result = Ok(state);
                    let generation = stop_generation.load(Ordering::Acquire);
                    if generation & 1 == 1 && state == MotionState::Stopped {
                        confirmed_stop_generation = Some(generation);
                    }
                } else {
                    last_stop_result = result.clone();
                }
                update_health(&health, &result);
                let _ = reply.send(result);
            }
            Job::EmergencyStop { reply } => {
                active_lease = None;
                let generation = stop_generation.load(Ordering::Acquire);
                if generation & 1 == 1 && handled_stop_generation != Some(generation) {
                    last_stop_result = send_stop(
                        &mut port,
                        &mut sequence,
                        &stop_generation,
                        &stop_notify,
                        &health,
                    )
                    .await;
                    if let Ok(state) = last_stop_result {
                        last_state = state;
                        if state == MotionState::Stopped {
                            confirmed_stop_generation = Some(generation);
                        }
                    }
                    handled_stop_generation = Some(generation);
                    last_stop_attempt = Instant::now();
                }
                let _ = reply.send(last_stop_result.clone());
            }
            Job::ClearEmergencyStop {
                generation,
                cancelled,
                reply,
            } => {
                let result = if cancelled.is_cancelled() {
                    Err(ControlError::Timeout)
                } else if active_lease.is_some() {
                    Err(ControlError::Unsafe)
                } else if generation & 1 == 0
                    || confirmed_stop_generation != Some(generation)
                    || stop_generation
                        .compare_exchange(
                            generation,
                            generation.wrapping_add(1),
                            Ordering::AcqRel,
                            Ordering::Acquire,
                        )
                        .is_err()
                {
                    Err(ControlError::Unsafe)
                } else {
                    confirmed_stop_generation = None;
                    handled_stop_generation = None;
                    last_stop_result = Ok(MotionState::Stopped);
                    Ok(())
                };
                let _ = reply.send(result);
            }
            Job::ReadState { reply } => {
                if active_lease.is_some_and(|active| Instant::now() >= active.expires_at) {
                    active_lease = None;
                    last_stop_result = send_stop(
                        &mut port,
                        &mut sequence,
                        &stop_generation,
                        &stop_notify,
                        &health,
                    )
                    .await;
                    if let Ok(state) = last_stop_result {
                        last_state = state;
                    }
                }
                let generation = stop_generation.load(Ordering::Acquire);
                let result = if generation & 1 == 1 {
                    if confirmed_stop_generation == Some(generation)
                        && last_state == MotionState::Stopped
                    {
                        Ok(MotionState::Stopped)
                    } else {
                        Err(ControlError::Unsafe)
                    }
                } else if active_lease.is_none() {
                    if last_state == MotionState::Stopped {
                        Ok(MotionState::Stopped)
                    } else {
                        Err(ControlError::Unsafe)
                    }
                } else {
                    exchange_next(
                        &mut port,
                        &mut sequence,
                        Command::Status,
                        &stop_generation,
                        &stop_notify,
                        None,
                        active_lease.map(|active| active.expires_at),
                    )
                    .await
                };
                if let Ok(state) = result {
                    last_state = state;
                    if state == MotionState::Stopped {
                        if let Some(active) = active_lease.as_mut() {
                            active.motion = None;
                        }
                    }
                } else {
                    active_lease = None;
                    if generation & 1 == 0 {
                        last_stop_result = send_stop(
                            &mut port,
                            &mut sequence,
                            &stop_generation,
                            &stop_notify,
                            &health,
                        )
                        .await;
                    }
                }
                update_health(&health, &result);
                let _ = reply.send(result);
            }
        }
    }
}

async fn send_stop(
    port: &mut SerialStream,
    sequence: &mut u16,
    stop_generation: &AtomicU64,
    stop_notify: &Notify,
    health: &Mutex<Health>,
) -> Result<MotionState, ControlError> {
    let result = exchange_next(
        port,
        sequence,
        Command::Stop,
        stop_generation,
        stop_notify,
        None,
        None,
    )
    .await;
    update_health(health, &result);
    result
}

async fn exchange_next(
    port: &mut SerialStream,
    sequence: &mut u16,
    command: Command,
    stop_generation: &AtomicU64,
    stop_notify: &Notify,
    cancelled: Option<&Cancellation>,
    valid_until: Option<Instant>,
) -> Result<MotionState, ControlError> {
    let seq = *sequence;
    *sequence = (*sequence).wrapping_add(1);
    exchange(
        port,
        seq,
        command,
        stop_generation,
        stop_notify,
        cancelled,
        valid_until,
    )
    .await
}

async fn exchange(
    port: &mut SerialStream,
    seq: u16,
    cmd: Command,
    stop_generation: &AtomicU64,
    stop_notify: &Notify,
    cancelled: Option<&Cancellation>,
    valid_until: Option<Instant>,
) -> Result<MotionState, ControlError> {
    if valid_until.is_some_and(|deadline| Instant::now() >= deadline) {
        return Err(ControlError::Expired);
    }
    if cancelled.is_some_and(Cancellation::is_cancelled) {
        return Err(ControlError::Timeout);
    }
    if is_latched(stop_generation) && cmd != Command::Stop {
        return Err(ControlError::Unsafe);
    }

    let mut payload = String::new();
    protocol::encode_request(&mut payload, Request { seq, command: cmd })
        .map_err(|e| ControlError::Transport(e.to_string()))?;
    match timeout(WRITE_TIMEOUT, port.write_all(payload.as_bytes())).await {
        Ok(Ok(())) => {}
        Ok(Err(error)) => return Err(ControlError::Transport(error.to_string())),
        Err(_) => return Err(ControlError::Timeout),
    }

    let deadline = Instant::now() + ACK_TIMEOUT;
    let mut line = Vec::with_capacity(64);
    loop {
        if valid_until.is_some_and(|valid_until| Instant::now() >= valid_until) {
            return Err(ControlError::Expired);
        }
        if cancelled.is_some_and(Cancellation::is_cancelled) {
            return Err(ControlError::Timeout);
        }
        if is_latched(stop_generation) && cmd != Command::Stop {
            return Err(ControlError::Unsafe);
        }

        let mut bytes = [0u8; 64];
        tokio::select! {
            biased;
            _ = async { if let Some(cancelled) = cancelled { cancelled.cancelled().await; } }, if cancelled.is_some() => {
                return Err(ControlError::Timeout);
            }
            _ = stop_notify.notified(), if cmd != Command::Stop => {
                if is_latched(stop_generation) {
                    return Err(ControlError::Unsafe);
                }
            }
            _ = tokio::time::sleep_until(tokio::time::Instant::from_std(deadline)) => {
                if valid_until.is_some_and(|valid_until| Instant::now() >= valid_until) {
                    return Err(ControlError::Expired);
                }
                return Err(ControlError::Timeout);
            }
            read = port.read(&mut bytes) => {
                match read {
                    Ok(0) => return Err(ControlError::Disconnected),
                    Ok(n) => {
                        for &byte in &bytes[..n] {
                            if byte == b'\n' {
                                if let Ok(s) = std::str::from_utf8(&line) {
                                    if let Ok(resp) = protocol::decode_response(s) {
                                        if resp.seq == seq {
                                            if valid_until.is_some_and(|valid_until| Instant::now() >= valid_until) {
                                                return Err(ControlError::Expired);
                                            }
                                            let state = validate_response(resp)?;
                                            if cmd == Command::Stop && state != MotionState::Stopped {
                                                return Err(ControlError::Unsafe);
                                            }
                                            return Ok(state);
                                        }
                                    }
                                }
                                line.clear();
                            } else if line.len() >= 63 {
                                line.clear();
                            } else if byte != b'\r' {
                                line.push(byte);
                            }
                        }
                    }
                    Err(error) => return Err(ControlError::Transport(error.to_string())),
                }
            }
        }
    }
}

fn validate_response(resp: Response) -> Result<MotionState, ControlError> {
    if !resp.ok {
        return Err(ControlError::Rejected);
    }
    Ok(to_domain(resp.state))
}

fn to_domain(state: protocol::MotionState) -> MotionState {
    match state {
        protocol::MotionState::Stopped => MotionState::Stopped,
        protocol::MotionState::Forward => MotionState::Forward,
        protocol::MotionState::Backward => MotionState::Backward,
    }
}

fn update_health(health: &Mutex<Health>, result: &Result<MotionState, ControlError>) {
    match result {
        Ok(_) => set_health(health, Health::Ready),
        Err(ControlError::Disconnected) => set_health(health, Health::Disconnected),
        Err(error) => set_health(health, Health::Degraded(error.to_string())),
    }
}

fn set_health(health: &Mutex<Health>, value: Health) {
    *health
        .lock()
        .unwrap_or_else(std::sync::PoisonError::into_inner) = value;
}

fn is_latched(generation: &AtomicU64) -> bool {
    generation.load(Ordering::Acquire) & 1 == 1
}

fn latch_stop(generation: &AtomicU64, stop_notify: &Notify) {
    let mut current = generation.load(Ordering::Acquire);
    loop {
        let next = if current & 1 == 0 {
            current.wrapping_add(1)
        } else {
            current.wrapping_add(2)
        };
        match generation.compare_exchange_weak(current, next, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => {
                stop_notify.notify_one();
                return;
            }
            Err(actual) => current = actual,
        }
    }
}
