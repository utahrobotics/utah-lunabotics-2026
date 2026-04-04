#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output},
    peripherals::USB,
    pwm::{Config, Pwm, SetDutyCycle},
    time_driver::init,
    usb::{Driver, InterruptHandler},
};
use embassy_time::{Duration, Timer};
use embassy_usb::{
    UsbDevice,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
};
use embedded_common::{
    Actuator, ActuatorCommand, Direction, FromPico, MAX_MESSAGE_SIZE, PicoError,
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

static PACKET_SIZE: u16 = 64;
const POLL_INTERVAL_MS: u64 = 100;

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

    // motor_flt pins
    let driver_fault_detectors: [FaultDetector<'_>; 3] = [
        FaultDetector {
            fault: Input::new(p.PIN_10, embassy_rp::gpio::Pull::Up),
            which: Actuator::Lift,
        },
        FaultDetector {
            fault: Input::new(p.PIN_16, embassy_rp::gpio::Pull::Up),
            which: Actuator::Bucket,
        },
        FaultDetector {
            fault: Input::new(p.PIN_22, embassy_rp::gpio::Pull::Up),
            which: Actuator::Dumper,
        },
    ];
    let actuators = [
        ActuatorDriver {
            sleep: Output::new(p.PIN_11, Level::Low),
            dir: Output::new(p.PIN_7, Level::Low),
            pwm: Pwm::new_output_a(p.PWM_SLICE4, p.PIN_8, Config::default()),
            which: Actuator::Lift,
        },
        ActuatorDriver {
            sleep: Output::new(p.PIN_17, Level::Low),
            dir: Output::new(p.PIN_13, Level::Low),
            pwm: Pwm::new_output_a(p.PWM_SLICE7, p.PIN_14, Config::default()),
            which: Actuator::Bucket,
        },
        ActuatorDriver {
            sleep: Output::new(p.PIN_23, Level::Low),
            dir: Output::new(p.PIN_19, Level::Low),
            pwm: Pwm::new_output_a(p.PWM_SLICE2, p.PIN_20, Config::default()),
            which: Actuator::Dumper,
        },
        // 4th motor driver slot is currently unused
        // ActuatorDriver {
        //     sleep: Output::new(p.PIN_29, Level::Low),
        //     fault: Input::new(p.PIN_28, embassy_rp::gpio::Pull::Up),
        //     dir: Output::new(p.PIN_25, Level::Low),
        //     pwm: Pwm::new_output_a(p.PWM_SLICE5, p.PIN_26, Config::default()),
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
            // for now just always spit out an error. from faults on an empty iterator will just return picoerror::other.
            let err = PicoError::from_faults(
                fault_detectors
                    .iter()
                    .filter(|fd| fd.is_faulted())
                    .map(|fd| fd.which),
            );
            let serialized = FromPico::Error(err).serialize();
            let len = cobs::encode(&serialized, &mut stuffed);
            for chunk in stuffed[..len + 1].chunks(16) {
                if let Err(e) = writer.write_packet(chunk).await {
                    error!("[COBS ERROR] {:?}", e);
                    break 'writer;
                }
            }

            // TODO: once secondary pico is implemented, read any sensor stuff off of it and send it over to the host here

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
        while let Ok(n) = reader.read_packet(&mut encoded_buf).await {
            let mut remaining = &encoded_buf[..n];
            while !remaining.is_empty() {
                match decoder.push(remaining) {
                    Ok(Some(report)) => {
                        let frame = &decoder.dest()[..report.frame_size()];
                        if let Ok(frame_bytes) = <[u8; ActuatorCommand::SIZE]>::try_from(frame) {
                            match ActuatorCommand::deserialize(frame_bytes) {
                                Ok(cmd) => {
                                    info!("received command: {}", cmd);
                                    match cmd {
                                        ActuatorCommand::SetSpeed(speed, actuator, direction) => {
                                            if let Some(a) =
                                                actuators.iter_mut().find(|a| a.which == actuator)
                                            {
                                                a.drive(speed, direction);
                                            }
                                        }
                                        ActuatorCommand::Shake => {
                                            warn!("Fuck you");
                                        }
                                        ActuatorCommand::StartPercuss => {
                                            warn!("Fuck you");
                                        }
                                        ActuatorCommand::StopPercuss => {
                                            warn!("Fuck you")
                                        }
                                        ActuatorCommand::StopAll => {
                                            actuators.iter_mut().for_each(|controller| {
                                                controller.sleep();
                                            });
                                        }
                                    }
                                }
                                Err(e) => warn!("failed to deserialize ActuatorCommand: {}", e),
                            }
                        } else {
                            warn!(
                                "unexpected frame size: {} (expected {})",
                                report.frame_size(),
                                ActuatorCommand::SIZE
                            );
                        }
                        // advance past consumed bytes, reset decoder for next frame
                        remaining = &remaining[report.parsed_size()..];
                        decoder = cobs::CobsDecoder::new(&mut decoded_buf);
                    }
                    Ok(None) => break, // decoder state fine, but more data needed
                    Err(_) => {
                        warn!("COBS decode error, resetting decoder");
                        decoder = cobs::CobsDecoder::new(&mut decoded_buf);
                        break;
                    }
                }
            }
        }
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;
#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}
