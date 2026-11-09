//! USB CDC driver for one Pico 2.
//! A dedicated thread exclusively owns the serial port, processes requests
//! sequentially, and sends a heartbeat even while stdin waits for input.
use robot_application::ports::{ControlError, MotorPort};
use robot_device_manager::Device;
use robot_domain::{Capability, Direction, Health, MotionState};
use robot_protocol::{self as protocol, Command, Request, Response};
use serialport::SerialPort;
use std::{
    io::{Read, Write},
    sync::{mpsc, Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

const HEARTBEAT: Duration = Duration::from_millis(300);
const ACK_TIMEOUT: Duration = Duration::from_millis(650);
const CAPABILITIES: &[Capability] = &[Capability::Motor];

type Reply = mpsc::Sender<Result<MotionState, ControlError>>;
struct Job { command: Command, reply: Reply }

pub struct PicoUsbMotor {
    id: String,
    tx: mpsc::Sender<Job>,
    health: Arc<Mutex<Health>>,
}

impl PicoUsbMotor {
    pub fn connect(id: impl Into<String>, path: &str) -> Result<Self, ControlError> {
        let port = serialport::new(path, 115_200)
            .timeout(Duration::from_millis(50))
            .open()
            .map_err(|e| ControlError::Transport(e.to_string()))?;
        let (tx, rx) = mpsc::channel();
        let health = Arc::new(Mutex::new(Health::Degraded("connecting".to_owned())));
        let worker_health = Arc::clone(&health);
        thread::spawn(move || serial_worker(port, rx, worker_health));
        Ok(Self { id: id.into(), tx, health })
    }

    fn execute(&self, command: Command) -> Result<MotionState, ControlError> {
        let (reply_tx, reply_rx) = mpsc::channel();
        self.tx.send(Job { command, reply: reply_tx })
            .map_err(|_| ControlError::Disconnected)?;
        reply_rx.recv_timeout(Duration::from_secs(2)).map_err(|err| match err {
            mpsc::RecvTimeoutError::Timeout => ControlError::Timeout,
            mpsc::RecvTimeoutError::Disconnected => ControlError::Disconnected,
        })?
    }
}

impl MotorPort for PicoUsbMotor {
    fn set_direction(&self, direction: Direction) -> Result<MotionState, ControlError> {
        self.execute(match direction {
            Direction::Forward => Command::Forward,
            Direction::Backward => Command::Backward,
        })
    }
    fn stop(&self) -> Result<MotionState, ControlError> { self.execute(Command::Stop) }
    fn read_state(&self) -> Result<MotionState, ControlError> { self.execute(Command::Status) }
}

impl Device for PicoUsbMotor {
    fn id(&self) -> &str { &self.id }
    fn capabilities(&self) -> &'static [Capability] { CAPABILITIES }
    fn health(&self) -> Health { self.health.lock().unwrap().clone() }
}

fn to_domain(state: protocol::MotionState) -> MotionState {
    match state {
        protocol::MotionState::Stopped => MotionState::Stopped,
        protocol::MotionState::Forward => MotionState::Forward,
        protocol::MotionState::Backward => MotionState::Backward,
    }
}

fn set_health(health: &Mutex<Health>, value: Health) {
    if let Ok(mut guard) = health.lock() { *guard = value; }
}

fn serial_worker(mut port: Box<dyn SerialPort>, rx: mpsc::Receiver<Job>, health: Arc<Mutex<Health>>) {
    let mut sequence: u16 = 1;
    loop {
        match rx.recv_timeout(HEARTBEAT) {
            Ok(job) => {
                let result = exchange(port.as_mut(), sequence, job.command);
                update_health(&health, &result);
                let _ = job.reply.send(result);
            }
            Err(mpsc::RecvTimeoutError::Timeout) => {
                let result = exchange(port.as_mut(), sequence, Command::Ping);
                update_health(&health, &result);
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => {
                let _ = exchange(port.as_mut(), sequence, Command::Stop);
                break;
            }
        }
        sequence = sequence.wrapping_add(1);
    }
    set_health(&health, Health::Disconnected);
}

fn update_health(health: &Mutex<Health>, result: &Result<MotionState, ControlError>) {
    match result {
        Ok(_) => set_health(health, Health::Ready),
        Err(e) => set_health(health, Health::Degraded(e.to_string())),
    }
}

fn exchange(port: &mut dyn SerialPort, seq: u16, cmd: Command) -> Result<MotionState, ControlError> {
    let mut payload = String::new();
    protocol::encode_request(&mut payload, Request { seq, command: cmd })
        .map_err(|e| ControlError::Transport(e.to_string()))?;
    port.write_all(payload.as_bytes())
        .map_err(|e| ControlError::Transport(e.to_string()))?;

    let deadline = Instant::now() + ACK_TIMEOUT;
    let mut line = Vec::with_capacity(64);
    while Instant::now() < deadline {
        let mut bytes = [0u8; 64];
        match port.read(&mut bytes) {
            Ok(n) => {
                for &byte in &bytes[..n] {
                    if byte == b'\n' {
                        if let Ok(s) = std::str::from_utf8(&line) {
                            if let Ok(resp) = protocol::decode_response(s) {
                                if resp.seq == seq { return validate_response(resp); }
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
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(ControlError::Transport(e.to_string())),
        }
    }
    Err(ControlError::Timeout)
}

fn validate_response(resp: Response) -> Result<MotionState, ControlError> {
    if !resp.ok { return Err(ControlError::Rejected); }
    Ok(to_domain(resp.state))
}
