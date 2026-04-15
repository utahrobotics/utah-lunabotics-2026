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
    UART0_IRQ => InterruptHandler<UART0>;
});

static PACKET_SIZE: u16 = 64;
const POLL_INTERVAL_MS: u64 = 100;

static SENSOR_READINGS: Channel<CriticalSectionRawMutex, SensorReading, 1> = Channel::new();

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
            which: Actuator::Lift,
        },
        FaultDetector {
            fault: Input::new(p.PIN_15, embassy_rp::gpio::Pull::Up),
            which: Actuator::Bucket,
        },
        FaultDetector {
            fault: Input::new(p.PIN_16, embassy_rp::gpio::Pull::Up),
            which: Actuator::Dumper,
        },
        // PIN_25 is Motor4 placeholder from Vincent's spec, omitted for now
    ];
    let actuators = [
        ActuatorDriver { // GPIO8 = physical pin 11, GPIO0 = pin 1
            sleep: Output::new(p.PIN_8, Level::Low),
            dir: Output::new(p.PIN_0, Level::Low),
            pwm: Pwm::new_output_a(p.PWM_SLICE4, p.PIN_8, Config::default()),
            which: Actuator::Lift,
        },
        ActuatorDriver {  // GPIO 9 and 2 = pin 12 and 4
            sleep: Output::new(p.PIN_9, Level::Low),
            dir: Output::new(p.PIN_2, Level::Low),
            pwm: Pwm::new_output_a(p.PWM_SLICE7, p.PIN_14, Config::default()),
            which: Actuator::Bucket,
        },
        ActuatorDriver { // GPIO 10 and 4 = pin 14 and 6
            sleep: Output::new(p.PIN_10, Level::Low),
            dir: Output::new(p.PIN_4, Level::Low),
            pwm: Pwm::new_output_a(p.PWM_SLICE2, p.PIN_20, Config::default()),
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

    let uart= {
        let mut cfg = UartConfig::default();
        cfg.baudrate = 115200;
        // GP12 and GP13 = physical 16 and 17, UART0 TX and RX
        Uart::new(p.UART0, p.PIN_12, p.PIN_13, Irqs, p.DMA_CH0, p.DMA_CH1);
    }
    static SECONDARY_UART: StaticCell<Uart<'static, UART0, Async>> = StaticCell::new();
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
            // for now just always spit out an error. from_faults on an empty iterator will just return picoerror::other.
            let err = PicoError::from_faults(
                fault_detectors
                    .iter()
                    .filter(|fd| fd.is_faulted())
                    .map(|fd| fd.which),
            );
            let serialized = FromPico::Error(err).serialize();
            let len = cobs::encode(&serialized, &mut stuffed);
            for chunk in stuffed[..len + 1].chunks(64) {
                if let Err(e) = writer.write_packet(chunk).await {
                    error!("[COBS ERROR] {:?}", e);
                    break 'writer;
                }
            }

            if let Ok(_sensors) = SENSOR_READINGS.try_receive(){
                // TODO: send sensor readings to host
                // let serialized = FromPico::Reading([FromIMU; 4], sensors).serialize();
                // let len = cobs::encode(&serialized, &mut stuffed);
                // for chunk in stuffed[..len + 1].chunks(64) {
                //     if let Err(e) = writer.write_packet(chunk).await {
                //         error!("[COBS ERROR] {:?}", e);
                //         break 'writer;
                //     }
                // }
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
                                            warn!("Shake unimplemented");
                                        }
                                        ActuatorCommand::StartPercuss => {
                                            warn!("Percussor unimplemented");
                                        }
                                        ActuatorCommand::StopPercuss => {
                                            warn!("Percussor unimplemented");
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

/// Constantly polls all 11 active MUX channels and publishes reading to SENSOR_READINGS
async fn secondary_poll_loop(uart: &'static mut Uart<'static, UART0, Async>){
    const RESPONSE_TIMEOUT_MS:u64  = 50;
    let mut readings = [0u16;  SensorReading:CHANNEL_COUNT];
    loop{
        for ch in 0u8..SensorReading::CHANNEL_COUNT as u8{
            let req = SecondaryRequest {mux_address:ch}.serialize();
            if uart.write(&req).await.is_err(){
                error!("failed to write to secondary pico");
                readings[ch as usize] = 0; // output 0 to avoid sending a completely empty reading
                continue;
            }
            let response_buf = [0u8; SecondaryResponse:SIZE];
            match with_timeout(
                Duration:from_millis(RESPONSE_TIMEOUT_MS);
                uart.read(&mut response_buf);
            )
            .await(
                Ok(Ok(_)) => readings[ch as usize] = SecondaryResponse.deserialize(response_buf).adc_value,
                Ok(Err(e)) => { error!("secondary UART read error ch {}: {:?}", ch, e); readings[ch as usize] = 0; }
                Err(_)     => { error!("secondary response timeout ch {}", ch);           readings[ch as usize] = 0; }
            )
        }

         let sensor = SensorReading {
            m1_cs:     readings[0],
            m2_cs:     readings[1],
            m3_cs:     readings[2],
            m4_cs:     readings[3],
            m1_therm:  readings[4],
            m2_therm:  readings[5],
            m3_therm:  readings[6],
            m4_therm:  readings[7],
            drive1_he: readings[8],
            drive2_he: readings[9],
            amb_therm: readings[10],
        };

        SENSOR_READINGS.try_send(sensor).ok();
        Timer::after(Duration::from_millis(POLL_INTERVAL_MS)).await;
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;
#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}
