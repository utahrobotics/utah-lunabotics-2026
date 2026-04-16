#![no_std]
#![no_main]

use core::net::Ipv4Addr;
use core::panic::PanicInfo;

use accelerometer::vector::F32x3;
use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::i2c::{Async, I2c};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::peripherals::{I2C0, USB, I2C1, ADC, PIN_28, PIN_26};
use embassy_rp::pio::{self, Pio};
use embassy_rp::adc::{Adc, Channel, Config};
use embassy_rp::adc;

use embassy_rp::usb::Driver;
use embassy_rp::{bind_interrupts, i2c, usb, Peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embassy_usb::{class::cdc_acm::{CdcAcmClass, Receiver, Sender, State}, UsbDevice};
use embedded_common::*;
use embedded_common::IMU_READING_DELAY_MS;
use lsm6dsox::accelerometer::Accelerometer;
use lsm6dsox::types::Error;
use lsm6dsox::*;
use rand::RngCore;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
mod motor;
use crate::motor::*;

use core::cell::RefCell;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    ADC_IRQ_FIFO => adc::InterruptHandler;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p: Peripherals = embassy_rp::init(Default::default());
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);
    
    let mut m2 = Motor::new_m2(p.PIN_17, p.PIN_14, p.PIN_16, p.PWM_SLICE0);
    let mut m1 = Motor::new_m1(p.PIN_10, p.PIN_15, p.PIN_9, p.PWM_SLICE4);
    let mut percussor = Output::new(p.PIN_21, Level::Low);


    // Create embassy-usb Config
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("USR");
        config.product = Some("V3PICO");
        config.serial_number = Some(TERI_PICO_SERIAL);
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    // ttyACM0 for writing sensor data over usb to lunabot
    let mut class = {
        static CLASS_STATE: StaticCell<State> = StaticCell::new();
        let state = CLASS_STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, 64)
    };

    // Build the builder.
    let usb = builder.build();

    // calls usb.run()
    spawner.spawn(usb_task(usb)).unwrap();
    
    spawner.spawn(okay_task()).unwrap();
    
    class.wait_connection().await;

    let (class_tx, class_rx) = class.split();

    
    m1.enable();
    m2.enable();
    
    let mut i2c = I2c::new_async(p.I2C0, p.PIN_1, p.PIN_0, Irqs, i2c::Config::default());
    static I2C: StaticCell<Mutex<CriticalSectionRawMutex,RefCell<I2c<I2C0, Async>>>> = StaticCell::new();
    let i2c = I2C.init(Mutex::new(RefCell::new(i2c)));
    static LSM0: StaticCell<Lsm6dsox<I2c<'_, I2C0, Async>, Delay>> = StaticCell::new();
    let lsm0 = LSM0.init(lsm6dsox::Lsm6dsox::new(i2c, SlaveAddress::High, Delay {}));
    static LSM1: StaticCell<Lsm6dsox<I2c<'_, I2C0, Async>, Delay>> = StaticCell::new();
    let lsm1 = LSM1.init(lsm6dsox::Lsm6dsox::new(i2c, SlaveAddress::Low, Delay {}));
    
    let mut i2c1 = I2c::new_async(p.I2C1, p.PIN_3, p.PIN_2, Irqs, i2c::Config::default());
    static I2C1: StaticCell<Mutex<CriticalSectionRawMutex,RefCell<I2c<I2C1, Async>>>> = StaticCell::new();
    let i2c1 = I2C1.init(Mutex::new(RefCell::new(i2c1)));
    static LSM2: StaticCell<Lsm6dsox<I2c<'_, I2C1, Async>, Delay>> = StaticCell::new();
    let lsm2 = LSM2.init(lsm6dsox::Lsm6dsox::new(i2c1, SlaveAddress::High, Delay {}));
    static LSM3: StaticCell<Lsm6dsox<I2c<'_, I2C1, Async>, Delay>> = StaticCell::new();
    let lsm3 = LSM3.init(lsm6dsox::Lsm6dsox::new(i2c1, SlaveAddress::Low, Delay {}));
    let mut imu0 = [lsm0, lsm1];
    let mut imu1 = [lsm2, lsm3];
    for imu in imu0.iter_mut() {
        match setup_lsm_i2c0(imu) {
            Ok(_id) => {
                info!("[SUCCESS] setup imu0");
            }
            Err(e) => {
                //error!("lsm failed to setup: {:?}", e);
            }
        }
    }
    for imu in imu1.iter_mut() {
        match setup_lsm_i2c1(imu) {
            Ok(_id) => {
                info!("[SUCCESS] setup imu1");
            }
            Err(e) => {
                //error!("lsm failed to setup: {:?}", e);
            }
        }
    }
    spawner.spawn(read_sensors_loop(imu0, imu1, IMU_READING_DELAY_MS, class_tx, p.PIN_26, p.PIN_28, p.ADC));
    spawner.spawn(motor_controller_loop(class_rx, m1, m2, percussor));
}

fn setup_lsm_i2c0(lsm: &mut Lsm6dsox<I2c<'_, I2C0, Async>, Delay>) -> Result<u8, lsm6dsox::Error> {
    lsm.setup()?;
    lsm.set_gyro_sample_rate(DataRate::Freq52Hz)?;
    lsm.set_gyro_scale(GyroscopeScale::Dps2000)?;
    lsm.set_accel_sample_rate(DataRate::Freq52Hz)?;
    lsm.set_accel_scale(AccelerometerScale::Accel4g)?;
    lsm.check_id().map_err(|e| {
        //error!("error checking id of lsm6dsox: {:?}", e);
        lsm6dsox::Error::NotSupported
    })
}

fn setup_lsm_i2c1(lsm: &mut Lsm6dsox<I2c<'_, I2C1, Async>, Delay>) -> Result<u8, lsm6dsox::Error> {
    lsm.setup()?;
    lsm.set_gyro_sample_rate(DataRate::Freq52Hz)?;
    lsm.set_gyro_scale(GyroscopeScale::Dps2000)?;
    lsm.set_accel_sample_rate(DataRate::Freq52Hz)?;
    lsm.set_accel_scale(AccelerometerScale::Accel4g)?;
    lsm.check_id().map_err(|e| {
        //error!("error checking id of lsm6dsox: {:?}", e);
        lsm6dsox::Error::NotSupported
    })
}

#[embassy_executor::task]
async fn okay_task() {
    Timer::after(Duration::from_secs(10)).await;
    info!("IMU is okay");
}


#[embassy_executor::task]
async fn usb_task(mut usb: UsbDevice<'static, Driver<'static, USB>>) -> ! {
    usb.run().await
}


/// uses lsm to read in sensor data, then sends AngularRate and AccelerationNorm over ttyACM*
/// also reads actuator lengths from pot and sends it over in the same message
#[embassy_executor::task]
async fn read_sensors_loop(
    mut imu0: [&'static mut Lsm6dsox<'static, I2c<'static, I2C0, Async>, Delay>; 2],
    mut imu1: [&'static mut Lsm6dsox<'static, I2c<'static, I2C1, Async>, Delay>; 2],
    delay_ms: u64,
    mut class: Sender<'static, Driver<'static, USB>>,
    pot: PIN_26, pot2: PIN_28, adc: ADC
) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(delay_ms));
    let mut channel = Channel::new_pin(pot, Pull::None);
    let mut channel2 = Channel::new_pin(pot2, Pull::None);
    let mut adc = Adc::new(adc, Irqs, Config::default());
    loop {
        //----- ACTUATORS -----
        // 148 ish when fully retracted, 3720 ish when fully extended
        const AVERAGING: usize = 10;
        let mut total = 0;
        let mut total2 = 0;
        let mut t1_count = 0;
        let mut t2_count = 0;
        
        for _ in 0..AVERAGING {
            if let Ok(reading) = adc.read(&mut channel2).await {
                t2_count += 1;
                total2 += reading;
            }
            if let Ok(reading) = adc.read(&mut channel).await {
                t1_count += 1;
                total += reading;
            }
        }

        let m1_reading = if t1_count != 0 {(total/t1_count as u16)} else {0}; 
        let m2_reading = if t2_count != 0 {(total2/t2_count as u16)} else {0};
        let actuator_readings = ActuatorReading {
            m1_reading,
            m2_reading,
        };

        // ----- IMU -----
        let mut imu0_readings = [FromIMU::Error; 2];
        for (i,imu) in imu0.iter_mut().enumerate() {
            let mut error_occured = false;
            let rate = match imu.angular_rate() {
                Ok(r) => {
                    Some(embedded_common::AngularRate {
                        x: r.x.as_radians_per_second() as f32,
                        y: r.y.as_radians_per_second() as f32,
                        z: r.z.as_radians_per_second() as f32
                    })
                }
                Err(e) => {
                    if Error::NoDataReady == e {
                    } else {
                        // //error!("failed to read gyro from imu0_{}",i);
                        error_occured = true;
                    }
                    None
                }
            };
            let accel = match imu.accel_norm() {
                Ok(F32x3 { x, y, z }) => {
                    // info!("accel: x: {}, y: {}, z: {} m/s normalized", x,y,z);
                    Some(AccelerationNorm { x, y, z })
                }
                Err(e) => {
                    if Some(&Error::NoDataReady) == e.cause() {
                    } else {
                        // //error!("failed to read accel: {:?}", e.cause());
                        error_occured = true;
                    }
                    None
                }
            };
            if let (Some(rate),Some(accel)) = (rate,accel) {
                imu0_readings[i] = FromIMU::Reading(rate,accel);
            } else if error_occured {
                imu0_readings[i] = FromIMU::Error;
            } else {
                imu0_readings[i] = FromIMU::NoDataReady;
            }
        }
        
        embassy_futures::yield_now().await;
        let mut imu1_readings = [FromIMU::Error; 2];
        for (i,imu) in imu1.iter_mut().enumerate() {
            let mut error_occured = false;
            let rate = match imu.angular_rate() {
                Ok(r) => {
                    Some(embedded_common::AngularRate {
                        x: r.x.as_radians_per_second() as f32,
                        y: r.y.as_radians_per_second() as f32,
                        z: r.z.as_radians_per_second() as f32
                    })
                }
                Err(e) => {
                    if Error::NoDataReady == e {
                    } else {
                        //error!("failed to read gyro from imu1_{}",i);
                        error_occured = true;
                    }
                    None
                }
            };
            let accel = match imu.accel_norm() {
                Ok(F32x3 { x, y, z }) => {
                    Some(AccelerationNorm { x, y, z })
                }
                Err(e) => {
                    if Some(&Error::NoDataReady) == e.cause() {
                    } else {
                        //error!("failed to read accel: {:?}", e.cause());
                        error_occured = true;
                    }
                    None
                }
            };
            
            if let (Some(rate),Some(accel)) = (rate,accel) {
                imu1_readings[i] = FromIMU::Reading(rate,accel);
            } else if error_occured {
                imu1_readings[i] = FromIMU::Error;
            } else {
                imu1_readings[i] = FromIMU::NoDataReady;
            }
        }
        
        let msg = FromPico::Reading(
            [imu0_readings[0], imu0_readings[1], imu1_readings[0],imu1_readings[1]],
            actuator_readings
        );

        if class.dtr() {
            let serialized: [u8; FromPico::SIZE] = msg.serialize();
            let mut stuffed = [0u8; cobs::max_encoding_length(FromPico::SIZE)+1];
            let len = cobs::encode(&serialized, &mut stuffed);
            for chunk in stuffed[..len+1].chunks(16) {
                if let Err(e) = class.write_packet(chunk).await {
                    error!("{:?}",e);
                }
            }
        } else {
            warn!("data terminal not ready");
        }
        ticker.next().await;
    }
}


const RAMP_RATE: u16 = 1000;
const RAMP_TICK_MS: u64 = 10;

fn step_toward(current: u16, target: u16) -> u16 {
    if current < target {
        current.saturating_add(RAMP_RATE).min(target)
    } else if current > target {
        current.saturating_sub(RAMP_RATE).max(target)
    } else {
        current
    }
}

#[embassy_executor::task(pool_size = 1)]
async fn motor_controller_loop(mut class: Receiver<'static, Driver<'static, USB>>, mut m1: Motor<'static>, mut m2: Motor<'static>, mut percussor: Output<'static>) {
    let mut m1_current: u16 = 0;
    let mut m1_target: u16 = 0;
    let mut m2_current: u16 = 0;
    let mut m2_target: u16 = 0;
    let mut ticker = Ticker::every(Duration::from_millis(RAMP_TICK_MS));

    loop {
        let mut cmd_buf = [0u8; 5];
        match embassy_futures::select::select(class.read_packet(&mut cmd_buf), ticker.next()).await {
            embassy_futures::select::Either::First(result) => {
                if let Ok(_) = result {
                    if let Ok(cmd) = embedded_common::ActuatorCommand::deserialize(cmd_buf) {
                        match cmd {
                            ActuatorCommand::SetSpeed(speed, actuator, direction) => {
                                match actuator {
                                    Actuator::Lift => {
                                        m1.set_direction(direction);
                                        m1_target = speed;
                                        info!("cmd: {:?}", cmd);
                                    }
                                    Actuator::Bucket => {
                                        m2.set_direction(direction);
                                        m2_target = speed;
                                        info!("cmd: {:?}", cmd);
                                    }
                                    Actuator::Dumper => {
                                        warn!("use of dumper on a teri pico.");
                                    }
                                }
                            }
                            ActuatorCommand::Shake => {
                                // m1.shake().await;
                                m1_current = 0;
                                m1_target = 0;
                            }
                            ActuatorCommand::StartPercuss => {
                                info!("starting percussor");
                                percussor.set_high();
                            }
                            ActuatorCommand::StopPercuss => {
                                info!("stopping percussor");
                                percussor.set_low();
                            }
                            ActuatorCommand::StopAll => {}
                        }
                    } else {
                        warn!("failed to deserialize actuator command: {:?}", cmd_buf);
                    }
                }
            }
            embassy_futures::select::Either::Second(_) => {}
        }

        // Ramp toward targets every iteration (stop immediately if target is 0)
        if m1_current != m1_target {
            m1_current = if m1_target == 0 { 0 } else { step_toward(m1_current, m1_target) };
            if let Err(_) = m1.set_speed(m1_current) {
                error!("couldnt set lifts speed: Pwm error");
            }
        }
        if m2_current != m2_target {
            m2_current = if m2_target == 0 { 0 } else { step_toward(m2_current, m2_target) };
            if let Err(_) = m2.set_speed(m2_current) {
                error!("couldnt set bucket speed: Pwm error");
            }
        }
    }
}
