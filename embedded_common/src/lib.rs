#![no_std]

#[cfg(feature = "std")]
extern crate std;

// When using defmt-03 feature, re-export defmt 0.3 under the name `defmt`
// so that derive(defmt::Format) works uniformly
#[cfg(feature = "defmt-03")]
extern crate defmt_03 as defmt;

use core::ops::Not;

pub const PRIME_PICO_SERIAL: &'static str = "USR-PICO-PRIME";
pub const SECONDARY_PICO_SERIAL: &'static str = "USR-PICO-SECONDARY";
pub const TERI_PICO_SERIAL: &'static str = "USR-PICO-TERI";

pub const IMU_READING_DELAY_MS: u64 = 10;
pub const MAX_MESSAGE_SIZE: usize = 265;

/// MUX channels for actuator potentiometers on the secondary pico
pub const POT_MUX_LIFT: u8 = 11;
pub const POT_MUX_BUCKET: u8 = 12;
pub const POT_MUX_DUMPER: u8 = 13;
pub const POT_CHANNEL_COUNT: usize = 3;

const _: () = assert!(
    FromIMU::SIZE <= MAX_MESSAGE_SIZE,
    "FromIMU exceeds MAX_MESSAGE_SIZE"
);
const _: () = assert!(
    ActuatorCommand::SIZE <= MAX_MESSAGE_SIZE,
    "ActuatorCommand exceeds MAX_MESSAGE_SIZE"
);
const _: () = assert!(
    FromPico::SIZE <= MAX_MESSAGE_SIZE,
    "FromPicoV3 exceeds MAX_MESSAGE_SIZE"
);

#[cfg(feature = "std")]
extern crate cu_bincode as bincode;

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum Direction {
    Forward = 0,
    Backward = 1,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[repr(u8)]
/// Used to specify which actuator a command is meant for.
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum Actuator {
    /// the lift
    Lift = 0,
    /// the bucket
    Bucket = 1,
    /// the dumper
    Dumper = 2,
    /// unused
    Motor4 = 3,
}

impl Actuator {
    pub const ALL: [Actuator; 4] = [
        Actuator::Lift,
        Actuator::Bucket,
        Actuator::Dumper,
        Actuator::Motor4,
    ];

    pub fn as_bit(self) -> u8 {
        1 << (self as u8)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]

pub enum ActuatorCommand {
    SetSpeed(u16, Actuator, Direction),
    /// Move an actuator to a target angle (radians), PID loop on the pico handles convergence
    SetAngle(Actuator, f32),
    SetLiftIK(f32),
    Shake,
    StartPercuss,
    StopPercuss,
    #[default]
    StopAll,
}

/// Potentiometer readings from the 3 actuators, fast-polled message
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub struct PotReading {
    pub lift: u16,
    pub bucket: u16,
    pub dumper: u16,
}

impl PotReading {
    pub const SIZE: usize = 6; // 3 x u16

    pub fn serialize(&self) -> [u8; Self::SIZE] {
        let mut bytes = [0u8; Self::SIZE];
        bytes[0..2].copy_from_slice(&self.lift.to_le_bytes());
        bytes[2..4].copy_from_slice(&self.bucket.to_le_bytes());
        bytes[4..6].copy_from_slice(&self.dumper.to_le_bytes());
        bytes
    }

    pub fn deserialize(bytes: [u8; Self::SIZE]) -> Self {
        Self {
            lift: u16::from_le_bytes([bytes[0], bytes[1]]),
            bucket: u16::from_le_bytes([bytes[2], bytes[3]]),
            dumper: u16::from_le_bytes([bytes[4], bytes[5]]),
        }
    }

    pub fn empty() -> Self {
        Self { lift: 0, bucket: 0, dumper: 0 }
    }
}

const _: () = assert!(
    PotReading::SIZE <= MAX_MESSAGE_SIZE,
    "PotReading exceeds MAX_MESSAGE_SIZE"
);

/// Calibration constants for a single linear actuator's potentiometer
/// converts raw ADC readings to actuator extension length, and then to joint angle
pub struct ActuatorCalibration {
    /// ADC reading when the actuator is fully retracted
    pub adc_min: u16,
    /// ADC reading when the actuator is fully extended
    pub adc_max: u16,
    /// Actuator stroke length in meters (max extension - min extension)
    pub stroke_m: f32,
    /// Retracted length of the actuator in meters (pivot-to-pivot when fully retracted)
    pub retracted_len_m: f32,
    /// Distance from the actuator mount pivot to the joint pivot on the fixed side (meters)
    pub mount_a: f32,
    /// Distance from the actuator mount pivot to the joint pivot on the moving side (meters)
    pub mount_b: f32,
    /// Angle offset in radians (angle of the joint when actuator is fully retracted)
    pub angle_at_retracted: f32,
}

/// Placeholder calibration for the Lift actuator
pub const LIFT_CAL: ActuatorCalibration = ActuatorCalibration {
    adc_min: 100,
    adc_max: 3900,
    stroke_m: 0.200,
    retracted_len_m: 0.300,
    mount_a: 0.150,
    mount_b: 0.200,
    angle_at_retracted: 0.0,
};

/// Placeholder calibration for the Bucket actuator
pub const BUCKET_CAL: ActuatorCalibration = ActuatorCalibration {
    adc_min: 100,
    adc_max: 3900,
    stroke_m: 0.200,
    retracted_len_m: 0.300,
    mount_a: 0.150,
    mount_b: 0.200,
    angle_at_retracted: 0.0,
};

/// Placeholder calibration for the Dumper actuator
pub const DUMPER_CAL: ActuatorCalibration = ActuatorCalibration {
    adc_min: 100,
    adc_max: 3900,
    stroke_m: 0.200,
    retracted_len_m: 0.300,
    mount_a: 0.150,
    mount_b: 0.200,
    angle_at_retracted: 0.0,
};

impl ActuatorCalibration {
    /// Convert a raw ADC value to actuator extension length (meters)
    pub fn adc_to_length(&self, adc: u16) -> f32 {
        let adc = adc.clamp(self.adc_min, self.adc_max);
        let t = (adc - self.adc_min) as f32 / (self.adc_max - self.adc_min) as f32;
        self.retracted_len_m + t * self.stroke_m
    }
}

/// Channel mapping:
///   0  = M1_CS        (Motor 1 current sense)
///   1  = M2_CS        (Motor 2 current sense)
///   2  = M3_CS        (Motor 3 current sense)
///   3  = M4_CS        (Motor 4 current sense)
///   4  = M1_THERM     (Motor 1 thermistor)
///   5  = M2_THERM     (Motor 2 thermistor)
///   6  = M3_THERM     (Motor 3 thermistor)
///   7  = M4_THERM     (Motor 4 thermistor)
///   8  = DRIVE1_HE    (Drive 1 hall-effect sensor)
///   9  = DRIVE2_HE    (Drive 2 hall-effect sensor)
///   10 = AMB_THERM    (Ambient thermistor)
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub struct SensorReading {
    pub m1_cs: u16,
    pub m2_cs: u16,
    pub m3_cs: u16,
    pub m4_cs: u16,
    pub m1_therm: u16,
    pub m2_therm: u16,
    pub m3_therm: u16,
    pub m4_therm: u16,
    pub drive1_he: u16,
    pub drive2_he: u16,
    pub amb_therm: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum FromIMU {
    Reading(AngularRate, AccelerationNorm),
    NoDataReady,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum FromPico {
    Reading([FromIMU; 4], SensorReading),
    Error(PicoError),
}

impl Default for FromPico {
    fn default() -> Self {
        Self::Error(PicoError::default())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum PicoError {
    #[default]
    Other,
    /// Bitfield of faulted drivers: bit 0 = Lift, bit 1 = Bucket, bit 2 = Dumper
    MotorDriverFault(u8),
    SecondaryPicoUartError,
    SecondaryPicoUartTimeout,
}

impl PicoError {
    /// Build a fault from an iterator of faulted actuators.
    /// returns None if the input iterator is empty
    pub fn from_faults(faults: impl Iterator<Item = Actuator>) -> Option<Self> {
        let bits = faults.fold(0u8, |acc, a| acc | a.as_bit());
        if bits == 0 {
            None
        } else {
            Some(PicoError::MotorDriverFault(bits))
        }
    }

    /// Check if a specific actuator is faulted.
    pub fn is_faulted(&self, actuator: Actuator) -> bool {
        match self {
            PicoError::MotorDriverFault(bits) => bits & actuator.as_bit() != 0,
            _ => false,
        }
    }
}

/// Radians per second
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub struct AngularRate {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Negative z = robot accelerating forward
///
/// In the default orientation, should be [0.0, -9.81, 0.0]
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub struct AccelerationNorm {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl AccelerationNorm {
    pub fn serialize(&self) -> [u8; 12] {
        let mut bytes = [0u8; 12];
        bytes[0..4].copy_from_slice(&self.x.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.y.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.z.to_le_bytes());
        bytes
    }

    pub fn deserialize(bytes: [u8; 12]) -> Result<Self, &'static str> {
        let x = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let y = f32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let z = f32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        Ok(Self { x, y, z })
    }
}

impl AngularRate {
    pub fn serialize(&self) -> [u8; 12] {
        let mut bytes = [0u8; 12];
        bytes[0..4].copy_from_slice(&self.x.to_le_bytes());
        bytes[4..8].copy_from_slice(&self.y.to_le_bytes());
        bytes[8..12].copy_from_slice(&self.z.to_le_bytes());
        bytes
    }

    pub fn deserialize(bytes: [u8; 12]) -> Result<Self, &'static str> {
        let x = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let y = f32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let z = f32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        Ok(Self { x, y, z })
    }
}

impl FromIMU {
    pub const SIZE: usize = 25;
    pub fn serialize(&self) -> [u8; 25] {
        let mut bytes = [0u8; 25];
        match self {
            FromIMU::Reading(rate, accel) => {
                bytes[0] = 0;
                bytes[1..=12].copy_from_slice(&rate.serialize());
                bytes[13..].copy_from_slice(&accel.serialize());
            }
            FromIMU::NoDataReady => {
                bytes[0] = 2;
            }
            FromIMU::Error => {
                bytes[0] = 3;
            }
        }
        bytes
    }

    pub fn deserialize(bytes: [u8; 25]) -> Result<Self, &'static str> {
        let rate_bytes: [u8; 12] = bytes[1..=12]
            .as_ref()
            .try_into()
            .map_err(|_| "failed to deserialize FromIMU")?;
        let accel_bytes: [u8; 12] = bytes[13..]
            .as_ref()
            .try_into()
            .map_err(|_| "failed to deserialize FromIMU")?;

        match bytes[0] {
            0 => {
                let rate = AngularRate::deserialize(rate_bytes)?;
                let accel = AccelerationNorm::deserialize(accel_bytes)?;
                Ok(FromIMU::Reading(rate, accel))
            }
            2 => Ok(FromIMU::NoDataReady),
            3 => Ok(FromIMU::Error),
            _ => Err("Invalid variant tag"),
        }
    }
}

impl ActuatorCommand {
    pub const SIZE: usize = 6;
    pub fn deserialize(bytes: [u8; Self::SIZE]) -> Result<Self, &'static str> {
        match bytes[0] {
            0 => {
                let actuator = if bytes[3] == Actuator::Lift as u8 {
                    Actuator::Lift
                } else if bytes[3] == Actuator::Bucket as u8 {
                    Actuator::Bucket
                } else if bytes[3] == Actuator::Dumper as u8 {
                    Actuator::Dumper
                } else {
                    return Err("Unknown actuator specifier");
                };
                let speed = u16::from_le_bytes(
                    bytes[1..=2]
                        .try_into()
                        .map_err(|_| "Wrong number of bytes in actuator command")?,
                );
                let direction = if bytes[4] == Direction::Backward as u8 {
                    Direction::Backward
                } else {
                    Direction::Forward
                };
                Ok(ActuatorCommand::SetSpeed(speed, actuator, direction))
            }
            2 => Ok(ActuatorCommand::Shake),
            3 => Ok(ActuatorCommand::StartPercuss),
            4 => Ok(ActuatorCommand::StopPercuss),
            5 => Ok(ActuatorCommand::StopAll),
            6 => {
                let actuator = if bytes[1] == Actuator::Lift as u8 {
                    Actuator::Lift
                } else if bytes[1] == Actuator::Bucket as u8 {
                    Actuator::Bucket
                } else if bytes[1] == Actuator::Dumper as u8 {
                    Actuator::Dumper
                } else {
                    return Err("Unknown actuator specifier in SetAngle");
                };
                let angle = f32::from_le_bytes(
                    bytes[2..6]
                        .try_into()
                        .map_err(|_| "Wrong number of bytes for angle")?,
                );
                Ok(ActuatorCommand::SetAngle(actuator, angle))
            }
            _ => Err("Invalid variant tag"),
        }
    }

    pub fn serialize(&self) -> [u8; Self::SIZE] {
        match self {
            ActuatorCommand::SetSpeed(speed, actuator, direction) => {
                let mut bytes = [0u8; Self::SIZE];
                bytes[0] = 0;
                bytes[1..=2].copy_from_slice(&speed.to_le_bytes());
                bytes[3] = *actuator as u8;
                bytes[4] = *direction as u8;
                bytes
            }
            ActuatorCommand::SetAngle(actuator, angle) => {
                let mut bytes = [0u8; Self::SIZE];
                bytes[0] = 6;
                bytes[1] = *actuator as u8;
                bytes[2..6].copy_from_slice(&angle.to_le_bytes());
                bytes
            }
            ActuatorCommand::SetLiftIK(angle) => {
                let mut bytes = [0u8; Self::SIZE];
                bytes[0] = 7;
                bytes[1..5].copy_from_slice(&angle.to_le_bytes());
                bytes
            }
            ActuatorCommand::Shake => {
                let mut bytes = [0u8; Self::SIZE];
                bytes[0] = 2;
                bytes
            }
            ActuatorCommand::StartPercuss => {
                let mut bytes = [0u8; Self::SIZE];
                bytes[0] = 3;
                bytes
            }
            ActuatorCommand::StopPercuss => {
                let mut bytes = [0u8; Self::SIZE];
                bytes[0] = 4;
                bytes
            }
            ActuatorCommand::StopAll => {
                let mut bytes = [0u8; Self::SIZE];
                bytes[0] = 5;
                bytes
            }
        }
    }

    /// Speed is a percentage of the max the actuator can go.
    pub fn set_speed(mut speed: f64, actuator: Actuator, direction: Direction) -> Self {
        speed = speed.clamp(0.0, 1.0);
        ActuatorCommand::SetSpeed((speed * u16::MAX as f64) as u16, actuator, direction)
    }

    /// little helper method to make the actuators move slower at outreach events
    pub fn apply_speed_factor(&mut self, factor: f32) {
        match self {
            ActuatorCommand::SetSpeed(speed, _actuator, _direction) => {
                *speed = (factor * (*speed as f32)) as u16;
            }
            // SetAngle targets are absolute angles, speed factor doesn't apply
            _ => {}
        }
    }

    /// Convenience constructor for SetAngle.
    pub fn set_angle(actuator: Actuator, angle_rad: f32) -> Self {
        ActuatorCommand::SetAngle(actuator, angle_rad)
    }
}

impl Not for Direction {
    type Output = Self;

    fn not(self) -> Self::Output {
        if self == Self::Forward {
            return Self::Backward;
        } else {
            return Self::Forward;
        }
    }
}


impl SensorReading {
    pub const CHANNEL_COUNT: usize = 11; // 11 active mux channels
    pub const SIZE: usize = Self::CHANNEL_COUNT * 2; // 2 bytes per channel
    pub fn serialize(&self) -> [u8; Self::SIZE] {
        let fields: [u16; Self::CHANNEL_COUNT] = [
            self.m1_cs,
            self.m2_cs,
            self.m3_cs,
            self.m4_cs,
            self.m1_therm,
            self.m2_therm,
            self.m3_therm,
            self.m4_therm,
            self.drive1_he,
            self.drive2_he,
            self.amb_therm,
        ];
        let mut bytes = [0u8; Self::SIZE];
        for (i, val) in fields.iter().enumerate() {
            bytes[i * 2..i * 2 + 2].copy_from_slice(&val.to_le_bytes());
        }
        bytes
    }

    pub fn deserialize(bytes: [u8; Self::SIZE]) -> Self {
        let read = |i: usize| u16::from_le_bytes([bytes[i * 2], bytes[i * 2 + 1]]);
        Self {
            m1_cs: read(0),
            m2_cs: read(1),
            m3_cs: read(2),
            m4_cs: read(3),
            m1_therm: read(4),
            m2_therm: read(5),
            m3_therm: read(6),
            m4_therm: read(7),
            drive1_he: read(8),
            drive2_he: read(9),
            amb_therm: read(10),
        }
    }

    pub fn empty() -> Self {
        Self {
            m1_cs: 0,
            m2_cs: 0,
            m3_cs: 0,
            m4_cs: 0,
            m1_therm: 0,
            m2_therm: 0,
            m3_therm: 0,
            m4_therm: 0,
            drive1_he: 0,
            drive2_he: 0,
            amb_therm: 0,
        }
    }
}
impl FromPico {
    /// 1 tag + 4 FromImu (4×25) + 1 SensorReading = 123 bytes
    pub const SIZE: usize = 1 + 4 * FromIMU::SIZE + SensorReading::SIZE;

    pub fn serialize(&self) -> [u8; Self::SIZE] {
        let mut bytes = [0u8; Self::SIZE];
        match self {
            FromPico::Reading(readings, sensors) => {
                bytes[0] = 0;
                for (i, r) in readings.iter().enumerate() {
                    let start = 1 + i * FromIMU::SIZE;
                    bytes[start..start + FromIMU::SIZE].copy_from_slice(&r.serialize());
                }
                bytes[Self::SIZE - SensorReading::SIZE..].copy_from_slice(&sensors.serialize());
            }
            FromPico::Error(err) => {
                bytes[0] = 3;
                match err {
                    PicoError::Other => bytes[1] = 0,
                    PicoError::MotorDriverFault(bits) => {
                        bytes[1] = 1;
                        bytes[2] = *bits;
                    }
                    PicoError::SecondaryPicoUartError => {
                        bytes[1] = 2;
                    }
                    PicoError::SecondaryPicoUartTimeout => {
                        bytes[1] = 3;
                    }
                }
            }
        }
        bytes
    }

    pub fn deserialize(bytes: [u8; Self::SIZE]) -> Result<Self, &'static str> {
        match bytes[0] {
            0 => {
                let mut readings = [FromIMU::Error; 4];
                for i in 0..4 {
                    let start = 1 + i * FromIMU::SIZE;
                    let imu_bytes: [u8; FromIMU::SIZE] = bytes[start..start + FromIMU::SIZE]
                        .try_into()
                        .map_err(|_| "imu slice")?;
                    readings[i] = FromIMU::deserialize(imu_bytes)?;
                }
                let sensor_bytes: [u8; SensorReading::SIZE] = bytes
                    [Self::SIZE - SensorReading::SIZE..]
                    .try_into()
                    .map_err(|_| "sensor slice")?;
                Ok(FromPico::Reading(
                    readings,
                    SensorReading::deserialize(sensor_bytes),
                ))
            }
            3 => {
                let err = match bytes[1] {
                    0 => PicoError::Other,
                    1 => PicoError::MotorDriverFault(bytes[2]),
                    2 => PicoError::SecondaryPicoUartError,
                    3 => PicoError::SecondaryPicoUartTimeout,
                    _ => return Err("invalid PicoError tag"),
                };
                Ok(FromPico::Error(err))
            }
            _ => Err("invalid FromPico tag"),
        }
    }
}

/// Request to secondary pico, asking for a reading from a specific mux channel
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub struct SecondaryRequest {
    pub mux_address: u8,
}

impl SecondaryRequest {
    pub const SIZE: usize = 1;
    pub const MAX_CHANNEL: u8 = 15;

    pub fn serialize(&self) -> [u8; 1] {
        [self.mux_address]
    }

    pub fn deserialize(bytes: [u8; 1]) -> Result<Self, &'static str> {
        if bytes[0] <= Self::MAX_CHANNEL {
            Ok(Self {
                mux_address: bytes[0],
            })
        } else {
            Err("invalid MUX address: must be 0-15")
        }
    }
}

/// Response from secondary pico, containing raw 12 bit ADC reading for all 11 channels

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(any(feature = "defmt", feature = "defmt-03"), derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub struct SecondaryResponse {
    pub adc_value: u16,
}

impl SecondaryResponse {
    pub const SIZE: usize = 2;

    pub fn serialize(&self) -> [u8; 2] {
        self.adc_value.to_le_bytes()
    }

    pub fn deserialize(bytes: [u8; 2]) -> Self {
        Self {
            adc_value: u16::from_le_bytes(bytes),
        }
    }
}

#[cfg(test)]
#[cfg(feature = "std")]
mod tests {
    use std::vec::Vec;

    use super::*;

    fn make_sensor_reading() -> SensorReading {
        SensorReading {
            m1_cs: 100,
            m2_cs: 200,
            m3_cs: 300,
            m4_cs: 400,
            m1_therm: 500,
            m2_therm: 600,
            m3_therm: 700,
            m4_therm: 800,
            drive1_he: 900,
            drive2_he: 1000,
            amb_therm: 1100,
        }
    }

    #[test]
    fn direction_not_operator() {
        assert_eq!(!Direction::Forward, Direction::Backward);
        assert_eq!(!Direction::Backward, Direction::Forward);
        assert_eq!(!!Direction::Forward, Direction::Forward);
    }

    fn roundtrip_angular_rate(x: f32, y: f32, z: f32) {
        let original = AngularRate { x, y, z };
        let bytes = original.serialize();
        let result = AngularRate::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    fn roundtrip_accel(x: f32, y: f32, z: f32) {
        let original = AccelerationNorm { x, y, z };
        let bytes = original.serialize();
        let result = AccelerationNorm::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    #[test]
    fn angular_rate_roundtrip() {
        roundtrip_angular_rate(0.0, 0.0, 0.0);
        roundtrip_angular_rate(1.5, -2.3, 0.001);
        roundtrip_angular_rate(f32::MAX, f32::MIN, f32::EPSILON);
    }

    #[test]
    fn acceleration_norm_roundtrip() {
        roundtrip_accel(0.0, -9.81, 0.0);
        roundtrip_accel(1.0, 2.0, 3.0);
        roundtrip_accel(f32::MAX, f32::MIN, f32::EPSILON);
    }

    #[test]
    fn from_imu_reading_roundtrip() {
        let rate = AngularRate {
            x: 0.1,
            y: 0.2,
            z: 0.3,
        };
        let accel = AccelerationNorm {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        };
        let original = FromIMU::Reading(rate, accel);
        let bytes = original.serialize();
        let result = FromIMU::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    #[test]
    fn from_imu_no_data_ready_roundtrip() {
        let original = FromIMU::NoDataReady;
        let bytes = original.serialize();
        let result = FromIMU::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    #[test]
    fn from_imu_error_roundtrip() {
        let original = FromIMU::Error;
        let bytes = original.serialize();
        let result = FromIMU::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    #[test]
    fn actuator_bit_uniqueness() {
        let bits: Vec<u8> = Actuator::ALL.iter().map(|a| a.as_bit()).collect();
        for i in 0..bits.len() {
            for j in 0..bits.len() {
                if i != j {
                    assert_eq!(
                        bits[i] & bits[j],
                        0,
                        "Actuators at index {i} and {j} share a fault bit"
                    );
                }
            }
        }
    }

    #[test]
    fn from_imu_invalid_tag() {
        let mut bytes = [0u8; FromIMU::SIZE];
        bytes[0] = 255;
        assert!(FromIMU::deserialize(bytes).is_err());
    }

    #[test]
    fn actuator_command_set_speed_roundtrip() {
        for actuator in Actuator::ALL {
            for direction in [Direction::Forward, Direction::Backward] {
                let v = ActuatorCommand::SetSpeed(12345, actuator, direction);
                assert_eq!(ActuatorCommand::deserialize(v.serialize()).unwrap(), v);
            }
        }
    }

    #[test]
    fn actuator_command_shake_roundtrip() {
        let original = ActuatorCommand::Shake;
        let bytes = original.serialize();
        let result = ActuatorCommand::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    #[test]
    fn actuator_command_percuss_roundtrip() {
        let start = ActuatorCommand::StartPercuss;
        assert_eq!(
            start,
            ActuatorCommand::deserialize(start.serialize()).unwrap()
        );

        let stop = ActuatorCommand::StopPercuss;
        assert_eq!(
            stop,
            ActuatorCommand::deserialize(stop.serialize()).unwrap()
        );
    }

    #[test]
    fn actuator_command_set_speed_helper() {
        let cmd = ActuatorCommand::set_speed(0.5, Actuator::Lift, Direction::Forward);
        let bytes = cmd.serialize();
        let result = ActuatorCommand::deserialize(bytes).unwrap();
        assert_eq!(cmd, result);
    }

    #[test]
    fn actuator_command_invalid_tag() {
        let bytes = [255, 0, 0, 0, 0, 0];
        assert!(ActuatorCommand::deserialize(bytes).is_err());
    }

    #[test]
    fn actuator_command_invalid_actuator() {
        let mut bytes =
            ActuatorCommand::SetSpeed(100, Actuator::Lift, Direction::Forward).serialize();
        bytes[3] = 255;
        assert!(ActuatorCommand::deserialize(bytes).is_err());
    }

    #[test]
    fn from_pico_v3_reading_roundtrip() {
        let rate = AngularRate {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };
        let accel = AccelerationNorm {
            x: 0.0,
            y: -9.81,
            z: 0.0,
        };
        let readings = [
            FromIMU::Reading(rate, accel),
            FromIMU::NoDataReady,
            FromIMU::Error,
            FromIMU::Reading(
                AngularRate {
                    x: -1.0,
                    y: 0.0,
                    z: 0.5,
                },
                AccelerationNorm {
                    x: 1.0,
                    y: 1.0,
                    z: 1.0,
                },
            ),
        ];
        let original = FromPico::Reading(readings, make_sensor_reading());
        let bytes = original.serialize();
        assert_eq!(FromPico::deserialize(bytes).unwrap(), original);
    }

    #[test]
    fn from_pico_v3_error_roundtrip() {
        let original = FromPico::Error(PicoError::Other);
        let bytes = original.serialize();
        let result = FromPico::deserialize(bytes).unwrap();
        assert_eq!(original, result);

        for actuator in Actuator::ALL {
            let original = FromPico::Error(PicoError::MotorDriverFault(actuator.as_bit()));
            let bytes = original.serialize();
            let result = FromPico::deserialize(bytes).unwrap();
            assert_eq!(original, result);
        }

        let multi = PicoError::from_faults([Actuator::Lift, Actuator::Dumper].into_iter()).unwrap();
        assert!(multi.is_faulted(Actuator::Lift));
        assert!(!multi.is_faulted(Actuator::Bucket));
        assert!(multi.is_faulted(Actuator::Dumper));
        let original = FromPico::Error(multi);
        let bytes = original.serialize();
        assert_eq!(FromPico::deserialize(bytes).unwrap(), original);
    }

    #[test]
    fn from_pico_v3_invalid_tag() {
        let mut bytes = [0u8; FromPico::SIZE];
        bytes[0] = 255;
        assert!(FromPico::deserialize(bytes).is_err());
    }

    #[test]
    fn from_pico_size_constant() {
        assert_eq!(FromPico::SIZE, 1 + 4 * FromIMU::SIZE + SensorReading::SIZE);
        assert_eq!(FromPico::SIZE, 123);
    }

    #[test]
    fn pico_error_other_is_never_faulted() {
        for actuator in Actuator::ALL {
            assert!(!PicoError::Other.is_faulted(actuator));
        }
    }

    #[test]
    fn sensor_reading_roundtrip() {
        let v = make_sensor_reading();
        assert_eq!(SensorReading::deserialize(v.serialize()), v);
    }

    #[test]
    fn sensor_reading_boundary_values() {
        for val in [0u16, u16::MAX] {
            let v = SensorReading {
                m1_cs: val,
                m2_cs: val,
                m3_cs: val,
                m4_cs: val,
                m1_therm: val,
                m2_therm: val,
                m3_therm: val,
                m4_therm: val,
                drive1_he: val,
                drive2_he: val,
                amb_therm: val,
            };
            assert_eq!(SensorReading::deserialize(v.serialize()), v);
        }
    }

    #[test]
    fn sensor_reading_field_order() {
        let v = make_sensor_reading();
        let bytes = v.serialize();
        assert_eq!(u16::from_le_bytes([bytes[0], bytes[1]]), 100, "ch0 m1_cs");
        assert_eq!(u16::from_le_bytes([bytes[2], bytes[3]]), 200, "ch1 m2_cs");
        assert_eq!(u16::from_le_bytes([bytes[4], bytes[5]]), 300, "ch2 m3_cs");
        assert_eq!(u16::from_le_bytes([bytes[6], bytes[7]]), 400, "ch3 m4_cs");
        assert_eq!(
            u16::from_le_bytes([bytes[8], bytes[9]]),
            500,
            "ch4 m1_therm"
        );
        assert_eq!(
            u16::from_le_bytes([bytes[10], bytes[11]]),
            600,
            "ch5 m2_therm"
        );
        assert_eq!(
            u16::from_le_bytes([bytes[12], bytes[13]]),
            700,
            "ch6 m3_therm"
        );
        assert_eq!(
            u16::from_le_bytes([bytes[14], bytes[15]]),
            800,
            "ch7 m4_therm"
        );
        assert_eq!(
            u16::from_le_bytes([bytes[16], bytes[17]]),
            900,
            "ch8 drive1_he"
        );
        assert_eq!(
            u16::from_le_bytes([bytes[18], bytes[19]]),
            1000,
            "ch9 drive2_he"
        );
        assert_eq!(
            u16::from_le_bytes([bytes[20], bytes[21]]),
            1100,
            "ch10 amb_therm"
        );
    }

    #[test]
    fn secondary_request_all_valid_channels() {
        for ch in 0u8..=15 {
            let req = SecondaryRequest { mux_address: ch };
            assert_eq!(SecondaryRequest::deserialize(req.serialize()).unwrap(), req);
        }
    }

    #[test]
    fn secondary_request_invalid_address() {
        for addr in 16u8..=255 {
            assert!(
                SecondaryRequest::deserialize([addr]).is_err(),
                "address {addr} should be rejected"
            );
        }
    }

    #[test]
    fn secondary_response_roundtrip() {
        for val in [0u16, 2048, 4095, u16::MAX] {
            let v = SecondaryResponse { adc_value: val };
            assert_eq!(SecondaryResponse::deserialize(v.serialize()), v);
        }
    }

    #[test]
    fn pot_reading_roundtrip() {
        let v = PotReading {
            lift: 1000,
            bucket: 2000,
            dumper: 3000,
        };
        assert_eq!(PotReading::deserialize(v.serialize()), v);
    }

    #[test]
    fn pot_reading_boundary_values() {
        for val in [0u16, u16::MAX] {
            let v = PotReading {
                lift: val,
                bucket: val,
                dumper: val,
            };
            assert_eq!(PotReading::deserialize(v.serialize()), v);
        }
    }

    #[test]
    fn actuator_command_set_angle_roundtrip() {
        for actuator in [Actuator::Lift, Actuator::Bucket, Actuator::Dumper] {
            for angle in [0.0f32, 1.5, -0.5, core::f32::consts::PI] {
                let cmd = ActuatorCommand::SetAngle(actuator, angle);
                let bytes = cmd.serialize();
                let result = ActuatorCommand::deserialize(bytes).unwrap();
                assert_eq!(cmd, result);
            }
        }
    }

    #[test]
    fn actuator_command_set_angle_helper() {
        let cmd = ActuatorCommand::set_angle(Actuator::Lift, 1.0);
        assert_eq!(cmd, ActuatorCommand::SetAngle(Actuator::Lift, 1.0));
        let bytes = cmd.serialize();
        assert_eq!(ActuatorCommand::deserialize(bytes).unwrap(), cmd);
    }

    #[test]
    fn actuator_calibration_adc_to_length() {
        let cal = ActuatorCalibration {
            adc_min: 0,
            adc_max: 4000,
            stroke_m: 0.200,
            retracted_len_m: 0.300,
            mount_a: 0.0,
            mount_b: 0.0,
            angle_at_retracted: 0.0,
        };
        // At min ADC, length = retracted
        assert!((cal.adc_to_length(0) - 0.300).abs() < 1e-5);
        // At max ADC, length = retracted + stroke
        assert!((cal.adc_to_length(4000) - 0.500).abs() < 1e-5);
        // At midpoint
        assert!((cal.adc_to_length(2000) - 0.400).abs() < 1e-5);
        // Clamped below min
        assert!((cal.adc_to_length(0) - 0.300).abs() < 1e-5);
    }
}
