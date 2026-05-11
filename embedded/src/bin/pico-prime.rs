#![no_std]
#![no_main]

extern crate libm;

use core::cell::Cell;
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output},
    peripherals::{UART0, USB},
    pwm::{Config, Pwm, SetDutyCycle},
    uart::{Async, Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart},
    usb::{Driver, InterruptHandler},
};
use embassy_sync::{
    blocking_mutex::{raw::CriticalSectionRawMutex, Mutex},
    channel::Channel,
};
use embassy_time::{Duration, Timer, with_timeout};
use embassy_usb::{
    UsbDevice,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
};
use embedded_common::{
    Actuator, ActuatorCommand, Direction, FromIMU, FromPico, MAX_MESSAGE_SIZE, PicoError,
    PotReading, SecondaryRequest, SecondaryResponse, SensorReading,
    POT_MUX_LIFT, POT_MUX_BUCKET, POT_MUX_DUMPER,
    LIFT_CAL, BUCKET_CAL, DUMPER_CAL,
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
    UART0_IRQ => UartInterruptHandler<UART0>;
});

static PACKET_SIZE: u16 = 64;
const POLL_INTERVAL_MS: u64 = 5;

const POT_POLL_INTERVAL_MS: u64 = 1;
/// How many pot-poll iterations between full sensor sweeps
const SENSOR_SWEEP_DIVISOR: u32 = 10;
/// Angle error (radians) below which the motor is stopped to prevent oscillation
const ANGLE_DEADBAND_RAD: f32 = 0.02;

static SENSOR_READINGS: Channel<CriticalSectionRawMutex, SensorReading, 1> = Channel::new();
static POT_READINGS: Channel<CriticalSectionRawMutex, PotReading, 1> = Channel::new();

static SECONDARY_PICO_ERRORS: Channel<CriticalSectionRawMutex, PicoError, 1> = Channel::new();

static PID_COMMANDS: Channel<CriticalSectionRawMutex, (u16, Actuator, Direction), 4> = Channel::new();

/// Written by usb_rx_loop on SetAngle/StopAll/SetSpeed, read by secondary_poll_loop for PID.
static LIFT_ANGLE_TARGET: Mutex<CriticalSectionRawMutex, Cell<Option<f32>>> =
    Mutex::new(Cell::new(None));
static BUCKET_ANGLE_TARGET: Mutex<CriticalSectionRawMutex, Cell<Option<f32>>> =
    Mutex::new(Cell::new(None));
static DUMPER_ANGLE_TARGET: Mutex<CriticalSectionRawMutex, Cell<Option<f32>>> =
    Mutex::new(Cell::new(None));
fn get_angle_target(actuator: Actuator) -> Option<f32> {
    match actuator {
        Actuator::Lift => LIFT_ANGLE_TARGET.lock(|c| c.get()),
        Actuator::Bucket => BUCKET_ANGLE_TARGET.lock(|c| c.get()),
        Actuator::Dumper => DUMPER_ANGLE_TARGET.lock(|c| c.get()),
        _ => None,
    }
}
fn set_angle_target(actuator: Actuator, target: Option<f32>) {
    match actuator {
        Actuator::Lift => LIFT_ANGLE_TARGET.lock(|c| c.set(target)),
        Actuator::Bucket => BUCKET_ANGLE_TARGET.lock(|c| c.set(target)),
        Actuator::Dumper => DUMPER_ANGLE_TARGET.lock(|c| c.set(target)),
        _ => {},
    }
}


fn clear_all_angle_targets() {
    LIFT_ANGLE_TARGET.lock(|c| c.set(None));
    BUCKET_ANGLE_TARGET.lock(|c| c.set(None));
    DUMPER_ANGLE_TARGET.lock(|c| c.set(None));
}
/// Simple PID controller for angle-targeting actuators.
struct PidController {
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    prev_error: f32,
    max_integral: f32,
}
impl PidController {
    const fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            max_integral: 1.0,
        }
    }
    fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }

        /// Returns a value that should be mapped to motor speed + direction.
    fn update(&mut self, error: f32, dt: f32) -> f32 {
        self.integral += error * dt;
        // Anti-windup clamp
        if self.integral > self.max_integral {
            self.integral = self.max_integral;
        } else if self.integral < -self.max_integral {
            self.integral = -self.max_integral;
        }
        let derivative = if dt > 0.0 {
            (error - self.prev_error) / dt
        } else {
            0.0
        };
        self.prev_error = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }
}
/// Convert a PID output to a (speed, direction) pair suitable for ActuatorDriver::drive.
fn pid_output_to_drive(output: f32) -> (u16, Direction) {
    let direction = if output >= 0.0 {
        Direction::Forward
    } else {
        Direction::Backward
    };
    // Map |output| (0.0..1.0 range after clamp) to u16 duty cycle
    let magnitude = output.abs().min(1.0);
    let speed = (magnitude * u16::MAX as f32) as u16;
    (speed, direction)
}

// I hope this is correct 
fn length_to_angle(length: f32, cal: &embedded_common::ActuatorCalibration) -> f32 {
    let a = cal.mount_a;
    let b = cal.mount_b;
    let c = length;
    // Law of cosines: c² = a² + b² - 2ab·cos(C)  =>  C = acos((a² + b² - c²) / (2ab))
    let numerator = a * a + b * b - c * c;
    let denominator = 2.0 * a * b;
    if denominator.abs() < 1e-9 {
        return cal.angle_at_retracted;
    }
    let cos_val = (numerator / denominator).max(-1.0).min(1.0);
    let angle = libm::acosf(cos_val);
    angle + cal.angle_at_retracted
}

struct FaultDetector<'a> {
    fault: Input<'a>,
    which: Actuator,
}

impl<'a> FaultDetector<'a> {
    fn is_faulted(&self) -> bool {
        self.fault.is_low()
    }
}

struct ActuatorDriver<'a> {
    sleep: Output<'a>,
    dir: Output<'a>,
    pwm: Pwm<'a>,
    which: Actuator,
}

impl<'a> ActuatorDriver<'a> {
    fn drive(&mut self, speed: u16, direction: Direction) {
        if speed != 0 {
            self.sleep.set_high();
        }
        match direction {
            Direction::Forward => self.dir.set_high(),
            Direction::Backward => self.dir.set_low(),
        }
        let _ = self.pwm.set_duty_cycle(speed);
    }

    fn sleep(&mut self) {
        let _ = self.pwm.set_duty_cycle(0);
        let _ = self.sleep.set_low();
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    // motor_flt pins - GPIO14, 15, 16 = physical pin 19, 20, 21
    let driver_fault_detectors: [FaultDetector<'_>; 3] = [
        FaultDetector {
            fault: Input::new(p.PIN_14, embassy_rp::gpio::Pull::Up),
            which: Actuator::Bucket,
        },
        FaultDetector {
            fault: Input::new(p.PIN_15, embassy_rp::gpio::Pull::Up),
            which: Actuator::Lift,
        },
        FaultDetector {
            fault: Input::new(p.PIN_16, embassy_rp::gpio::Pull::Up),
            which: Actuator::Dumper,
        },
        // PIN_25 is Motor4 placeholder from Vincent's spec, omitted for now
    ];
    let actuators = [
        ActuatorDriver {
            // GPIO8 = physical pin 11, GPIO0 = pin 1
            sleep: Output::new(p.PIN_8, Level::Low),
            dir: Output::new(p.PIN_0, Level::Low),
            pwm: Pwm::new_output_b(p.PWM_SLICE0, p.PIN_1, Config::default()),
            which: Actuator::Bucket,
        },
        ActuatorDriver {
            // GPIO 9 and 2 = pin 12 and 4
            sleep: Output::new(p.PIN_9, Level::Low),
            dir: Output::new(p.PIN_2, Level::Low),
            pwm: Pwm::new_output_b(p.PWM_SLICE1, p.PIN_3, Config::default()),
            which: Actuator::Lift,
        },
        ActuatorDriver {
            // GPIO 10 and 4 = pin 14 and 6
            sleep: Output::new(p.PIN_10, Level::Low),
            dir: Output::new(p.PIN_4, Level::Low),
            pwm: Pwm::new_output_b(p.PWM_SLICE2, p.PIN_5, Config::default()),
            which: Actuator::Dumper,
        },
        // 4th motor driver slot is currently unused
        // ActuatorDriver { // GPIO 15 and 9
        //     sleep: Output::new(p.PIN_15, Level::Low),
        //     fault: Input::new(p.PIN_25, embassy_rp::gpio::Pull::Up),
        //     dir: Output::new(p.PIN_9, Level::Low),
        //     pwm: Pwm::new_output_a(p.PWM_SLICE5, p.PIN_7, Config::default()),
        //     which: ...,
        // },
    ];

    let driver = Driver::new(p.USB, Irqs);
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("USR");
        config.product = Some(embedded_common::PRIME_PICO_SERIAL);
        config.serial_number = Some(embedded_common::PRIME_PICO_SERIAL);
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [],
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    let class: CdcAcmClass<'_, Driver<'_, USB>> = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, PACKET_SIZE)
    };

    let usb = builder.build();

    info!("about to spawn usb task");
    spawner.spawn(usb_task(usb)).unwrap();

    let uart = {
        let mut cfg = UartConfig::default();
        cfg.baudrate = 115200;
        Uart::new(p.UART0, p.PIN_12, p.PIN_13, Irqs, p.DMA_CH0, p.DMA_CH1, cfg)
    };
    static SECONDARY_UART: StaticCell<Uart<'static, Async>> = StaticCell::new();
    let uart = SECONDARY_UART.init(uart);
    spawner.spawn(secondary_poll_loop(uart)).unwrap();

    let (class_tx, class_rx) = class.split();

    static ACTUATORS: StaticCell<[ActuatorDriver<'static>; 3]> = StaticCell::new();
    static FAULT_DETECTORS: StaticCell<[FaultDetector<'static>; 3]> = StaticCell::new();
    let actuators = ACTUATORS.init(actuators);
    let fault_detectors = FAULT_DETECTORS.init(driver_fault_detectors);
    spawner.spawn(usb_rx_loop(class_rx, actuators)).unwrap();
    spawner
        .spawn(usb_tx_loop(class_tx, fault_detectors))
        .unwrap();
    core::future::pending::<()>().await;
    defmt::unreachable!()
}

/// writes messages from pico to host
#[embassy_executor::task]
async fn usb_tx_loop(
    mut writer: Sender<'static, Driver<'static, USB>>,
    fault_detectors: &'static [FaultDetector<'static>; 3],
) {
    loop {
        writer.wait_connection().await;
        'writer: while writer.dtr() {
            let mut stuffed = [0u8; cobs::max_encoding_length(FromPico::SIZE) + 1];
            let err = PicoError::from_faults(
                fault_detectors
                    .iter()
                    .filter(|fd| fd.is_faulted())
                    .map(|fd| fd.which),
            );
            if let Some(fault_error) = err {
                let serialized = FromPico::Error(fault_error).serialize();
                let len = cobs::encode(&serialized, &mut stuffed);
                for chunk in stuffed[..len + 1].chunks(64) {
                    if let Err(e) = writer.write_packet(chunk).await {
                        error!("[COBS ERROR] {:?}", e);
                        break 'writer;
                    }
                }
            }

            if let Ok(secondary_pico_err) = SECONDARY_PICO_ERRORS.try_receive() {
                let serialized = FromPico::Error(secondary_pico_err).serialize();
                let len = cobs::encode(&serialized, &mut stuffed);
                for chunk in stuffed[..len + 1].chunks(64) {
                    if let Err(e) = writer.write_packet(chunk).await {
                        error!("[COBS ERROR] {:?}", e);
                        break 'writer;
                    }
                }
            }

            if let Ok(sensors) = SENSOR_READINGS.try_receive() {
                let serialized = FromPico::Reading([FromIMU::NoDataReady; 4], sensors).serialize();
                let len = cobs::encode(&serialized, &mut stuffed);
                for chunk in stuffed[..len + 1].chunks(64) {
                    if let Err(e) = writer.write_packet(chunk).await {
                        error!("[COBS ERROR] {:?}", e);
                        break 'writer;
                    }
                }
            }

            // Send pot readings as a separate COBS frame
            if let Ok(pots) = POT_READINGS.try_receive() {
                let serialized = pots.serialize();
                let mut pot_stuffed = [0u8; cobs::max_encoding_length(PotReading::SIZE) + 1];
                let len = cobs::encode(&serialized, &mut pot_stuffed);
                for chunk in pot_stuffed[..len + 1].chunks(64) {
                    if let Err(e) = writer.write_packet(chunk).await {
                        error!("[COBS ERROR] {:?}", e);
                        break 'writer;
                    }
                }
            }

            Timer::after(Duration::from_millis(POLL_INTERVAL_MS)).await;
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

/// reads commands from the host, and controlls actuators accordingly
#[embassy_executor::task]
async fn usb_rx_loop(
    mut reader: Receiver<'static, Driver<'static, USB>>,
    actuators: &'static mut [ActuatorDriver<'static>; 3],
) {
    let mut decoded_buf = [0u8; MAX_MESSAGE_SIZE];
    let mut encoded_buf = [0u8; PACKET_SIZE as usize];
    let mut decoder = cobs::CobsDecoder::new(&mut decoded_buf);
    loop {
        reader.wait_connection().await;
        'connected: loop {
            // Wait for either a USB packet or a PID drive command
            match select(
                reader.read_packet(&mut encoded_buf),
                PID_COMMANDS.receive(),
            )
            .await
            {
                // USB packet arrived
                Either::First(Ok(n)) => {
                    let mut remaining = &encoded_buf[..n];
                    while !remaining.is_empty() {
                        match decoder.push(remaining) {
                            Ok(Some(report)) => {
                                let frame = &decoder.dest()[..report.frame_size()];
                                if let Ok(frame_bytes) =
                                    <[u8; ActuatorCommand::SIZE]>::try_from(frame)
                                {
                                    match ActuatorCommand::deserialize(frame_bytes) {
                                        Ok(cmd) => {
                                            info!("received command: {}", cmd);
                                            match cmd {
                                                ActuatorCommand::SetSpeed(
                                                    speed,
                                                    actuator,
                                                    direction,
                                                ) => {
                                                    // Manual speed cancels angle target
                                                    set_angle_target(actuator, None);
                                                    if let Some(a) = actuators
                                                        .iter_mut()
                                                        .find(|a| a.which == actuator)
                                                    {
                                                        a.drive(speed, direction);
                                                    }
                                                }
                                                ActuatorCommand::SetAngle(actuator, angle) => {
                                                    info!(
                                                        "SetAngle: actuator={}, angle={}",
                                                        actuator, angle
                                                    );
                                                    set_angle_target(actuator, Some(angle));
                                                }
                                                ActuatorCommand::Shake => {
                                                    warn!("Shake unimplemented");
                                                }
                                                ActuatorCommand::StartPercuss => {
                                                    warn!("Percussor unimplemented");
                                                }
                                                ActuatorCommand::StopPercuss => {
                                                    warn!("Percussor unimplemented");
                                                }
                                                ActuatorCommand::StopAll => {
                                                    clear_all_angle_targets();
                                                    actuators.iter_mut().for_each(|controller| {
                                                        controller.sleep();
                                                    });
                                                }
                                            }
                                        }
                                        Err(e) => {
                                            warn!("failed to deserialize ActuatorCommand: {}", e)
                                        }
                                    }
                                } else {
                                    warn!(
                                        "unexpected frame size: {} (expected {})",
                                        report.frame_size(),
                                        ActuatorCommand::SIZE
                                    );
                                }
                                remaining = &remaining[report.parsed_size()..];
                                decoder = cobs::CobsDecoder::new(&mut decoded_buf);
                            }
                            Ok(None) => break,
                            Err(_) => {
                                warn!("COBS decode error, resetting decoder");
                                decoder = cobs::CobsDecoder::new(&mut decoded_buf);
                                break;
                            }
                        }
                    }
                }
                // USB read error — disconnected
                Either::First(Err(_)) => {
                    break 'connected;
                }
                // PID command from the poll loop
                Either::Second((speed, actuator, direction)) => {
                    if let Some(a) = actuators.iter_mut().find(|a| a.which == actuator) {
                        a.drive(speed, direction);
                    }
                }
            }
        }
        
    }
}

/// Read a single MUX channel from the secondary pico. Returns None on UART error or timeout.
async fn read_mux_channel(
    uart: &mut Uart<'static, Async>,
    ch: u8,
    timeout_ms: u64,
) -> Option<u16> {
    let req = SecondaryRequest { mux_address: ch }.serialize();
    if uart.write(&req).await.is_err() {
        error!("failed to write to secondary pico ch {}", ch);
        return None;
    }
    let mut response_buf = [0u8; SecondaryResponse::SIZE];
    match with_timeout(
        Duration::from_millis(timeout_ms),
        uart.read(&mut response_buf),
    )
    .await
    {
        Ok(Ok(_)) => Some(SecondaryResponse::deserialize(response_buf).adc_value),
        Ok(Err(e)) => {
            error!("secondary UART read error ch {}: {:?}", ch, e);
            let _ = SECONDARY_PICO_ERRORS.try_send(PicoError::SecondaryPicoUartError);
            None
        }
        Err(_) => {
            error!("secondary response timeout ch {}", ch);
            let _ = SECONDARY_PICO_ERRORS.try_send(PicoError::SecondaryPicoUartTimeout);
            None
        }
    }
}

/// Polls the secondary pico for sensor data.
/// - Every iteration (~1ms): reads 3 pot channels, publishes PotReading, runs PID.
/// - Every SENSOR_SWEEP_DIVISOR-th iteration: also reads one sensor channel.
#[embassy_executor::task]
async fn secondary_poll_loop(uart: &'static mut Uart<'static, Async>) {
    const RESPONSE_TIMEOUT_MS: u64 = 50;
    const POT_CHANNELS: [u8; 3] = [POT_MUX_LIFT, POT_MUX_BUCKET, POT_MUX_DUMPER];
    const PID_DT: f32 = POT_POLL_INTERVAL_MS as f32 / 1000.0;

    // Placeholder PID gains — tune on real hardware
    let mut pid_lift = PidController::new(2.0, 0.1, 0.05);
    let mut pid_bucket = PidController::new(2.0, 0.1, 0.05);
    let mut pid_dumper = PidController::new(2.0, 0.1, 0.05);

    let mut sensor_readings = [0u16; SensorReading::CHANNEL_COUNT];
    let mut sensor_ch_idx: u8 = 0;
    let mut iteration: u32 = 0;

    loop {
        // --- Always: read pot channels (3 UART roundtrips) ---
        let mut pot_raw = [0u16; 3];
        for (i, &ch) in POT_CHANNELS.iter().enumerate() {
            pot_raw[i] = read_mux_channel(uart, ch, RESPONSE_TIMEOUT_MS).await.unwrap_or(0);
        }

        let pot = PotReading {
            lift: pot_raw[0],
            bucket: pot_raw[1],
            dumper: pot_raw[2],
        };
        info!("POT lift={} bucket={} dumper={}", pot_raw[0], pot_raw[1], pot_raw[2]);
        POT_READINGS.try_send(pot).ok();

        // --- Run PID for each actuator that has an angle target ---
        for (actuator, raw, cal, pid) in [
            (Actuator::Lift, pot_raw[0], &LIFT_CAL, &mut pid_lift),
            (Actuator::Bucket, pot_raw[1], &BUCKET_CAL, &mut pid_bucket),
            (Actuator::Dumper, pot_raw[2], &DUMPER_CAL, &mut pid_dumper),
        ] {
            if let Some(target_angle) = get_angle_target(actuator) {
                let length = cal.adc_to_length(raw);
                let current_angle = length_to_angle(length, cal);
                let error = target_angle - current_angle;

                // Dead-band: stop motor and reset integrator when close enough
                if error.abs() < ANGLE_DEADBAND_RAD {
                    pid.reset();
                    PID_COMMANDS.try_send((0, actuator, Direction::Forward)).ok();
                } else {
                    let output = pid.update(error, PID_DT);
                    let (speed, direction) = pid_output_to_drive(output);
                    PID_COMMANDS.try_send((speed, actuator, direction)).ok();
                }
            } else {
                pid.reset();
            }
        }

        // --- Periodically: read one sensor channel per iteration ---
        if iteration % SENSOR_SWEEP_DIVISOR == 0
            && sensor_ch_idx < SensorReading::CHANNEL_COUNT as u8
        {
            if let Some(val) = read_mux_channel(uart, sensor_ch_idx, RESPONSE_TIMEOUT_MS).await {
                sensor_readings[sensor_ch_idx as usize] = val;
            } else {
                sensor_readings[sensor_ch_idx as usize] = 0;
            }
            sensor_ch_idx += 1;

            if sensor_ch_idx >= SensorReading::CHANNEL_COUNT as u8 {
                let sensor = SensorReading {
                    m1_cs: sensor_readings[0],
                    m2_cs: sensor_readings[1],
                    m3_cs: sensor_readings[2],
                    m4_cs: sensor_readings[3],
                    m1_therm: sensor_readings[4],
                    m2_therm: sensor_readings[5],
                    m3_therm: sensor_readings[6],
                    m4_therm: sensor_readings[7],
                    drive1_he: sensor_readings[8],
                    drive2_he: sensor_readings[9],
                    amb_therm: sensor_readings[10],
                };
                info!("sensor reading: {:?}", sensor);
                SENSOR_READINGS.try_send(sensor).ok();
                sensor_ch_idx = 0;
            }
        }

        iteration = iteration.wrapping_add(1);
        Timer::after(Duration::from_millis(POT_POLL_INTERVAL_MS)).await;
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;
#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}
