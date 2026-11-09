#![no_std]
#![no_main]

use core::fmt::{self, Write};
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use panic_probe as _;
use robot_protocol::{self as protocol, Command, MotionState, Response};
use rp235x_hal as hal;
use usb_device::{class_prelude::*, prelude::*};
use usbd_serial::SerialPort;

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

const XTAL_HZ: u32 = 12_000_000;
const STEP_INTERVAL_US: u64 = 5_000;
const WATCHDOG_US: u64 = 1_500_000;
const STEPS: [u8; 8] = [1, 3, 2, 6, 4, 12, 8, 9];

// Fixed-size output buffer; no heap allocation on the RP2350.
struct Frame {
    bytes: [u8; 64],
    len: usize,
    sent: usize,
}
impl Frame {
    const fn new() -> Self {
        Self {
            bytes: [0; 64],
            len: 0,
            sent: 0,
        }
    }
    fn clear(&mut self) {
        self.len = 0;
        self.sent = 0;
    }
    fn pending(&self) -> bool {
        self.sent < self.len
    }
}
impl Write for Frame {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        let data = s.as_bytes();
        if self.len + data.len() > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..self.len + data.len()].copy_from_slice(data);
        self.len += data.len();
        Ok(())
    }
}

#[hal::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    // ULN2003 IN1..IN4 -> Pico 2 GP2..GP5.
    let mut in1 = pins.gpio2.into_push_pull_output();
    let mut in2 = pins.gpio3.into_push_pull_output();
    let mut in3 = pins.gpio4.into_push_pull_output();
    let mut in4 = pins.gpio5.into_push_pull_output();
    let _ = in1.set_low();
    let _ = in2.set_low();
    let _ = in3.set_low();
    let _ = in4.set_low();

    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USB,
        pac.USB_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut serial = SerialPort::new(&usb_bus);
    // Example-only VID/PID, not an allocated production USB vendor ID.
    let mut usb = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .strings(&[StringDescriptors::default()
            .manufacturer("Robot Rust")
            .product("Pico2 Motor")
            .serial_number("PICO2-MOTOR-001")])
        .unwrap()
        .device_class(2)
        .build();

    let mut state = MotionState::Stopped;
    let mut phase = 0usize;
    let mut last_command = timer.get_counter().ticks();
    let mut last_step = last_command;
    let mut received = [0u8; 64];
    let mut received_len = 0usize;
    let mut discard_line = false;
    let mut pending = Frame::new();

    loop {
        // Keep the USB stack alive even while the motor rotates.
        if usb.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            if let Ok(count) = serial.read(&mut buf) {
                for &byte in &buf[..count] {
                    if byte == b'\n' {
                        if !discard_line && !pending.pending() {
                            let response = match core::str::from_utf8(&received[..received_len])
                                .ok()
                                .and_then(|s| protocol::decode_request(s).ok())
                            {
                                Some(req) => {
                                    let now = timer.get_counter().ticks();
                                    match req.command {
                                        Command::Forward => state = MotionState::Forward,
                                        Command::Backward => state = MotionState::Backward,
                                        Command::Stop => state = MotionState::Stopped,
                                        Command::Status | Command::Ping => {}
                                    }
                                    last_command = now;
                                    Response {
                                        seq: req.seq,
                                        ok: true,
                                        state,
                                    }
                                }
                                None => Response {
                                    seq: 0,
                                    ok: false,
                                    state,
                                },
                            };
                            pending.clear();
                            let _ = protocol::encode_response(&mut pending, response);
                        }
                        received_len = 0;
                        discard_line = false;
                    } else if byte != b'\r' && !discard_line {
                        if received_len < received.len() {
                            received[received_len] = byte;
                            received_len += 1;
                        } else {
                            discard_line = true;
                        }
                    }
                }
            }
        }

        // Retry partially transmitted replies, without blocking the motor loop.
        if pending.pending() {
            if let Ok(written) = serial.write(&pending.bytes[pending.sent..pending.len]) {
                pending.sent += written;
            }
        }

        let now = timer.get_counter().ticks();
        if state != MotionState::Stopped && now.wrapping_sub(last_command) >= WATCHDOG_US {
            state = MotionState::Stopped;
        }

        if state == MotionState::Stopped {
            // De-energize coils; holding torque is intentionally not maintained.
            let _ = in1.set_low();
            let _ = in2.set_low();
            let _ = in3.set_low();
            let _ = in4.set_low();
        } else if now.wrapping_sub(last_step) >= STEP_INTERVAL_US {
            last_step = now;
            phase = match state {
                MotionState::Forward => (phase + 1) % 8,
                MotionState::Backward => (phase + 7) % 8,
                MotionState::Stopped => phase,
            };
            let pattern = STEPS[phase];
            if pattern & 1 != 0 {
                let _ = in1.set_high();
            } else {
                let _ = in1.set_low();
            }
            if pattern & 2 != 0 {
                let _ = in2.set_high();
            } else {
                let _ = in2.set_low();
            }
            if pattern & 4 != 0 {
                let _ = in3.set_high();
            } else {
                let _ = in3.set_low();
            }
            if pattern & 8 != 0 {
                let _ = in4.set_high();
            } else {
                let _ = in4.set_low();
            }
        }
    }
}
