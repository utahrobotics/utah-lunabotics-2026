// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

#![no_std]

mod accel;
mod gyro;
pub mod register;
pub mod types;

use core::{cell::RefCell, hint::black_box};
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, Mutex};
pub use register::*;
pub use types::*;

pub use accelerometer;
use accelerometer::{
    vector::{F32x3, I16x3},
    Accelerometer, RawAccelerometer,
};
use byteorder::{ByteOrder, LittleEndian};
use embedded_hal::{delay::DelayNs, i2c::I2c};
use enumflags2::BitFlags;


/// Representation of a LSM6DSOX. Stores the address and device peripherals.
pub struct Lsm6dsox<'a, I2C, Delay>
where
    I2C: I2c,
    Delay: DelayNs,
{
    delay: Delay,
    config: Configuration,
    registers: RegisterAccess<'a, I2C>,
}

impl<'a, I2C, Delay> Lsm6dsox<'a, I2C, Delay>
where
    I2C: I2c,
    Delay: DelayNs,
{
    pub fn dummy_angular_rate(&self) -> Result<AngularRate, Error> {
        let mut x = 0;
        black_box(while x <= 90000 {
            black_box(x += 1);
        });
        Ok(AngularRate {
            x: measurements::AngularVelocity::from_rpm(1.1).into(),
            y: measurements::AngularVelocity::from_rpm(2.2).into(),
            z: measurements::AngularVelocity::from_rpm(3.3).into(),
        })
    }
    
    pub fn dummy_accel_norm(&self) -> Result<F32x3, accelerometer::Error<Error>> {
        let mut x = 0;
        black_box(while x <= 90000 {
            black_box(x += 1);
        });
        Ok(F32x3 {
            x: 1.5,
            y: 2.5,
            z: 3.5,
        })
    }
    /// Create a new instance of this sensor.
    pub fn new(i2c: &'a Mutex<CriticalSectionRawMutex, RefCell<I2C>>, address: SlaveAddress, delay: Delay) -> Self {
        Self {
            delay,
            config: Configuration {
                xl_odr: DataRate::PowerDown,
                xl_scale: AccelerometerScale::Accel2g,
                g_odr: DataRate::PowerDown,
                g_scale: GyroscopeScale::Dps250,
            },
            registers: RegisterAccess::new(i2c, address),
        }
    }

    /// Destroy the sensor and return the hardware peripherals
    pub fn destroy(self) -> (&'a Mutex<CriticalSectionRawMutex, RefCell<I2C>>, Delay) {
        (self.registers.destroy(), self.delay)
    }

    /// Check whether the configured Sensor returns its correct id.
    ///
    /// Returns `Ok(id)` if `id` matches the Standard LSM6DSOX id,
    /// `Err(Some(id))` or `Err(None)` if `id` doesn't match or couldn't be read.
    pub fn check_id(&mut self) -> Result<u8, Option<u8>> {
        match self.registers.read_reg(PrimaryRegister::WHO_AM_I) {
            Ok(val) => {
                if val == 0x6C {
                    Ok(val)
                } else {
                    Err(Some(val))
                }
            }
            Err(_) => Err(None),
        }
    }

    /// Initializes the sensor.
    
    /// A software reset is performed and common settings are applied. The accelerometer and
    /// gyroscope are initialized with [`DataRate::PowerDown`].
    pub fn setup(&mut self) -> Result<(), Error> {
        self.update_reg_command(Command::SwReset)?;
        // Give it 5 tries
        // A delay is necessary here, otherwise reset may never finish because the lsm is too busy.
        let mut ctrl3_c_val = 0xFF;
        for _ in 0..5 {
            self.delay.delay_ms(10);
            ctrl3_c_val = self.registers.read_reg(PrimaryRegister::CTRL3_C)?;
            if ctrl3_c_val & 1 == 0 {
                break;
            }
        }

        if ctrl3_c_val & 1 != 0 {
            Err(Error::ResetFailed)
        } else {
            /* Disable I3C interface */
            self.registers
                .update_reg(PrimaryRegister::CTRL9_XL, 0x02, 0x02)?;
            self.registers
                .update_reg(PrimaryRegister::I3C_BUS_AVB, 0x00, 0b0001_1000)?;

            /* Enable Block Data Update */
            self.registers
                .update_reg(PrimaryRegister::CTRL3_C, 0b0100_0000, 0b0100_0000)?;

            self.set_accel_sample_rate(self.config.xl_odr)?;
            self.set_accel_scale(self.config.xl_scale)?;
            self.set_gyro_sample_rate(self.config.g_odr)?;
            self.set_gyro_scale(self.config.g_scale)?;

            /* Wait stable output */
            self.delay.delay_ms(100);

            Ok(())
        }
    }

    /// Checks the interrupt status of all possible sources.
    ///
    /// The interrupt flags will be cleared after this check, or according to the LIR mode of the specific source.
    pub fn check_interrupt_sources(&mut self) -> Result<BitFlags<InterruptCause>, Error> {
        let all_int_src = self.registers.read_reg(PrimaryRegister::ALL_INT_SRC)?;
        let flags = BitFlags::from_bits(all_int_src).map_err(|_| Error::InvalidData)?;

        Ok(flags)
    }

    /// Sets both Accelerometer and Gyroscope in power-down mode.
    pub fn power_down_mode(&mut self) -> core::result::Result<(), Error> {
        self.update_reg_command(Command::SetDataRateXl(DataRate::PowerDown))?;
        self.config.xl_odr = DataRate::PowerDown;

        self.update_reg_command(Command::SetDataRateG(DataRate::PowerDown))?;
        self.config.g_odr = DataRate::PowerDown;

        Ok(())
    }

    /// Maps an available interrupt source to a available interrupt line.
    ///
    /// Toggles whether a interrupt source will generate interrupts on the specified line.
    ///
    /// Note: Interrupt sources [SHUB](InterruptSource::SHUB) and [Timestamp](InterruptSource::Timestamp) are not available on both [interrupt lines](InterruptLine).
    ///
    /// Interrupts need to be enabled globally for a mapping to take effect. See [`Lsm6dsox::enable_interrupts()`].
    pub fn map_interrupt(
        &mut self,
        int_src: InterruptSource,
        int_line: InterruptLine,
        active: bool,
    ) -> Result<(), types::Error> {
        // TODO track interrupt mapping state in config
        //  This would allow us to automatically enable or disable interrupts globally.

        match (int_line, int_src) {
            (InterruptLine::INT1, InterruptSource::Timestamp) => Err(Error::NotSupported),
            (InterruptLine::INT2, InterruptSource::SHUB) => Err(Error::NotSupported),
            (_, _) => self.update_reg_command(Command::MapInterrupt(int_line, int_src, active)),
        }
    }

    /// Enable basic interrupts
    ///
    /// Enables/disables interrupts for 6D/4D, free-fall, wake-up, tap, inactivity.
    pub fn enable_interrupts(&mut self, enabled: bool) -> Result<(), Error> {
        self.update_reg_command(Command::InterruptEnable(enabled))
    }

    /// Updates a register according to a given [`Command`].
    fn update_reg_command(&mut self, command: Command) -> Result<(), Error> {
        self.registers
            .update_reg(command.register(), command.bits(), command.mask())
    }

    // pub unsafe fn register_access(&mut self) -> &mut RegisterAccess<I2C> {
    //     &mut self.registers
    // }
}
