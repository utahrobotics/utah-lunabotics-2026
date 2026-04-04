// Copyright Open Logistics Foundation
//
// Licensed under the Open Logistics Foundation License 1.3.
// For details on the licensing terms, see the LICENSE file.
// SPDX-License-Identifier: OLFL-1.3

//! Types used by the sensor.
//!
//! Structs and Enums representing the sensors configuration, readings and states.

use defmt::Format;
use enumflags2::bitflags;
use measurements::AngularVelocity;
use num_enum::TryFromPrimitive;
use embedded_hal::i2c::I2c;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use core::cell::RefCell;

/// Lsm6dsox errors
#[derive(Clone, Copy, Debug, Format, PartialEq)]
pub enum Error {
    I2cWriteError,
    I2cReadError,
    ResetFailed,
    NoDataReady,
    InvalidData,
    NotSupported,
}
/// Angular rate measurement result
///
/// Holds three [AngularVelocity] measurements.
#[derive(Clone, Copy, Debug)]
pub struct AngularRate {
    pub x: AngularVelocity,
    pub y: AngularVelocity,
    pub z: AngularVelocity,
}

/// Raw Angular rate measurement result
///
/// Holds three [i16] measurements.

// TODO: maybe use a micromath vector here
// (be aware of dependency hell, the accelerometer crate uses an old version of micromath)
#[derive(Clone, Copy, Debug)]
pub struct RawAngularRate {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

/// Bitflags to represent interrupts causes
///
/// Reports which sources triggered an interrupt,
/// which hasn't been cleared yet.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
#[bitflags]
#[repr(u8)]
pub enum InterruptCause {
    TimestampEndcount = 0b1000_0000,
    SleepChange = 0b0010_0000,
    D6d = 0b0001_0000,
    DoubleTap = 0b0000_1000,
    SingleTap = 0b0000_0100,
    WakeUp = 0b0000_0010,
    FreeFall = 0b0000_0001,
}

/// Tap source register
///
/// Holds information about tap events.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[bitflags]
#[repr(u8)]
pub enum TapSource {
    /// Tap event detection status.
    TapIA = 0b0100_0000,
    /// Single-tap event status.
    SingleTap = 0b0010_0000,
    /// Double-tap event status.
    DoubleTap = 0b0001_0000,
    /// Sign of acceleration detected by tap event.
    ///
    /// Present when negative sign of acceleration detected by tap event.
    NegativeTapSign = 0b0000_1000,
    /// Tap event detection status on X-axis.
    XTap = 0b0000_0100,
    /// Tap event detection status on Y-axis.
    YTap = 0b0000_0010,
    /// Tap event detection status on Z-axis.
    ZTap = 0b0000_0001,
}

/// I2C Address of the sensor in use
///
/// The address can be set via the sensors address pin (*SDO/SA0*).
#[derive(Clone, Copy)]
pub enum SlaveAddress {
    /// *SDO/SA0* pulled Low
    Low = 0b110_1010,
    /// *SDO/SA0* pulled High
    High = 0b110_1011,
}

/// Sensor configuration state
#[derive(Clone, Copy)]
pub(crate) struct Configuration {
    // Would it be better to use Option types here
    // to be able to have uninitialized config fields?
    /// Accelerometer sampling rate or [`DataRate::PowerDown`]
    pub xl_odr: DataRate,
    /// Accelerometer maximum output range
    pub xl_scale: AccelerometerScale,
    /// Gyroscope sampling rate or [`DataRate::PowerDown`]
    ///
    /// Make sure not to use [`DataRate::Freq1Hz6`] here which is invalid for the gyroscope.
    pub g_odr: DataRate,
    /// Gyroscope maximum output range
    pub g_scale: GyroscopeScale,
}

/// Possible output data rates for both Accelerometer and Gyroscope
///
/// ## Power mode
/// **If _"high performance mode"_ is disabled**  for the Accelerometer or Gyroscope,
/// it will, depending on the selected data rate, operate in either:
/// - *"low power mode"*
/// - *"normal mode"*
/// - *"high performance mode"*
///
/// **Otherwise** it will always run in *"high performance mode"* (which is the default).
///
/// This enum corresponds to registers `CTRL1_XL` and `CTRL2_G`.
/// See the [Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf) for more information on data rates.
#[allow(dead_code)]
#[derive(Clone, Copy, TryFromPrimitive)]
#[repr(u8)]
pub enum DataRate {
    /// Power down state
    PowerDown = 0b0000_0000,
    /// 1.6 Hz
    ///
    /// Only available for Accelerometer.
    /// *"Low power mode"* only, defaults to 12 Hz when in *"high performance mode"*.
    Freq1Hz6 = 0b1011_0000,
    /// 12.5 Hz
    ///
    /// (low power)
    Freq12Hz5 = 0b0001_0000,
    /// 26 Hz
    ///
    /// (low power)
    Freq26Hz = 0b0010_0000,
    /// 52 Hz
    ///
    /// (low power)
    Freq52Hz = 0b0011_0000,
    /// 104 Hz
    ///
    /// (normal mode)
    Freq104Hz = 0b0100_0000,
    /// 208 Hz
    ///
    /// (normal mode)
    Freq208Hz = 0b0101_0000,
    /// 416 Hz
    ///
    /// (high performance)
    Freq416Hz = 0b0110_0000,
    /// 833 Hz
    ///
    /// (high performance)
    Freq833Hz = 0b0111_0000,
    /// 1.66 kHz
    ///
    /// (high performance)
    Freq1660Hz = 0b1000_0000,
    /// 3.33 kHz
    ///
    /// (high performance)
    Freq3330Hz = 0b1001_0000,
    /// 6.66 kHz
    ///
    /// (high performance)
    Freq6660Hz = 0b1010_0000,
}

impl From<DataRate> for f32 {
    fn from(data_rate: DataRate) -> f32 {
        match data_rate {
            DataRate::PowerDown => 0.0,
            DataRate::Freq1Hz6 => 1.6,
            DataRate::Freq12Hz5 => 12.5,
            DataRate::Freq26Hz => 26.0,
            DataRate::Freq52Hz => 52.0,
            DataRate::Freq104Hz => 104.0,
            DataRate::Freq208Hz => 208.0,
            DataRate::Freq416Hz => 416.0,
            DataRate::Freq833Hz => 833.0,
            DataRate::Freq1660Hz => 1660.0,
            DataRate::Freq3330Hz => 3330.0,
            DataRate::Freq6660Hz => 6660.0,
        }
    }
}

/// Possible accelerometer output ranges
///
/// g-force which can be reported by the accelerometer.
/// Measurements are reported as [i16] by the accelerometer mapping the configured scale.
///
/// Note: Values are provided in 16 bit 2nd complement values by the lsm6dsox,
/// which represent the configured scale range.
/// Meaning a configured scale of 16g would yield [i16::MAX] for 16g of acceleration.

// Corresponding to register ```CTRL1_XL```
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum AccelerometerScale {
    /// ±2g accelerometer Scale
    Accel2g = 0b0000_0000,
    /// ±16g accelerometer Scale
    ///
    /// When ```XL_FS_MODE``` in ```CTRL8_XL``` is set to 1, ```FS_XL_16g``` sets scale to 2g.
    Accel16g = 0b0000_0100,
    /// ±4g accelerometer Scale
    Accel4g = 0b0000_1000,
    /// ±8g accelerometer Scale
    Accel8g = 0b0000_1100,
}

impl AccelerometerScale {
    /// Returns the factor needed to convert a [i16] of the specified range to a float.
    ///
    /// Multiplying a given [i16] with the corresponding factor, will yield the measured g-force.
    pub fn to_factor(&self) -> f32 {
        match self {
            AccelerometerScale::Accel2g => 0.000061,
            AccelerometerScale::Accel16g => 0.0006714,
            AccelerometerScale::Accel4g => 0.000122,
            AccelerometerScale::Accel8g => 0.000244,
        }
    }
}

/// Possible gyroscope output ranges
///
/// Degree per second which can be reported by the gyroscope.
/// Measurements are reported as [i16] by the gyroscope mapping the configured scale.

// Corresponding to register ```CTRL2_G```
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum GyroscopeScale {
    /// ±125dps gyroscope scale
    Dps125 = 0b0000_0010,
    /// ±250dps gyroscope scale
    Dps250 = 0b0000_0000,
    /// ±500dps gyroscope scale
    Dps500 = 0b0000_0100,
    /// ±1000dps gyroscope scale
    Dps1000 = 0b0000_1000,
    /// ±2000dps gyroscope scale
    Dps2000 = 0b0000_1100,
}

impl GyroscopeScale {
    /// Returns the factor needed to convert a [i16] of the specified range to a float.
    ///
    /// Note: The values have been copied from the official ST driver.
    /// They correspond nicely to the scarce examples from the application note,
    /// but otherwise don't make much sense (for example calculate `125d/0x7FFF` and you'll get `0.003814`, slightly different than `0.004375`).
    /// On the other hand factors used for [AccelerometerScale] seem to be correct in the official ST driver and can be calculated as shown above.
    ///
    /// Also see this [GitHub issue](https://github.com/STMicroelectronics/lsm6dsox/issues/2).
    pub fn to_factor(&self) -> f32 {
        match self {
            GyroscopeScale::Dps125 => 0.004375,
            GyroscopeScale::Dps250 => 0.008750,
            GyroscopeScale::Dps500 => 0.01750,
            GyroscopeScale::Dps1000 => 0.0350,
            GyroscopeScale::Dps2000 => 0.070,
        }
    }
}

/// Generic axis enum to map various commands.
#[derive(Clone, Copy)]
pub enum Axis {
    X,
    Y,
    Z,
}

/// Configures the sensor to detect either only single-taps or both single- and double-taps.
///
/// Note: Mapping of the tap detection to **interrupt lines** is **independent** from this.
/// While double-taps can't be detected *without* detecting single-taps, they **can** be routed to interrupt lines separately (and also checked separately).
#[derive(Clone, Copy)]
pub enum TapMode {
    Single,
    SingleAndDouble,
}

/// Tap configuration
///
/// Configure tap recognition for x, y, z axis and set latched interrupts
#[derive(Clone, Copy)]
pub struct TapCfg {
    /// Enable tap detection on separate axes.
    pub en_x_y_z: (bool, bool, bool),
    /// Latched interrupts
    ///
    /// When set, the interrupt source has to be checked to clear the interrupt.
    pub lir: bool,
}

/// Representation of interrupt pins 1 and 2.
#[derive(Clone, Copy)]
pub enum InterruptLine {
    INT1,
    INT2,
}

/// Interrupt sources which can be routed to interrupt pins.
#[derive(Clone, Copy)]
pub enum InterruptSource {
    /// Activity/inactivity recognition events
    SleepChange,
    /// Single-tap recognition events
    SingleTap,
    /// Wakeup events
    WakeUp,
    /// Free-fall events
    FreeFall,
    /// Double-tap recognition events
    DoubleTap,
    /// 6D events
    D6d,
    /// Embedded functions events
    EmbeddedFunctions,
    /// Sensor hub communication concluded events
    ///
    /// Note: Only available on [InterruptLine::INT1]
    SHUB,
    /// Alert for timestamp overflows
    ///
    /// Reported within 6.4ms.
    ///
    /// Note: Only available on [InterruptLine::INT2]
    Timestamp,
}
