#![no_std]

use core::ops::Not;

pub const PRIME_PICO_SERIAL: &'static str = "USR-PICO-PRIME";
pub const SECONDARY_PICO_SERIAL: &'static str = "USR-PICO-SECONDARY";
pub const TERI_PICO_SERIAL: &'static str = "USR-PICO-TERI";




pub const IMU_READING_DELAY_MS: u64 = 10;
pub const MAX_MESSAGE_SIZE: usize = 265;

const _: () = assert!(FromIMU::SIZE <= MAX_MESSAGE_SIZE, "FromIMU exceeds MAX_MESSAGE_SIZE");
const _: () = assert!(ActuatorCommand::SIZE <= MAX_MESSAGE_SIZE, "ActuatorCommand exceeds MAX_MESSAGE_SIZE");
const _: () = assert!(FromPicoV3::SIZE <= MAX_MESSAGE_SIZE, "FromPicoV3 exceeds MAX_MESSAGE_SIZE");


#[cfg(feature="std")]
extern crate cu_bincode as bincode;



#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum Direction {
    Forward = 0,
    Backward = 1,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    Dumper = 2
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]

pub enum ActuatorCommand {
    SetSpeed(u16, Actuator, Direction),
    Shake,
    StartPercuss,
    #[default]
    StopPercuss,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// adc readings
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub struct ActuatorReading {
    pub m1_reading: u16,
    pub m2_reading: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum FromIMU {
    Reading(AngularRate, AccelerationNorm),
    NoDataReady,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[cfg_attr(
    feature = "std",
    derive(serde::Serialize, serde::Deserialize, bincode::Encode, bincode::Decode)
)]
pub enum FromPicoV3 {
    Reading([FromIMU; 4], ActuatorReading),
    #[default]
    Error,
}

/// Radians per second
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    pub const SIZE: usize = 5;
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
            ActuatorCommand::Shake => {
                let mut bytes = [0u8; 5];
                bytes[0] = 2;
                bytes
            }
            ActuatorCommand::StartPercuss => {
                let mut bytes = [0u8; 5];
                bytes[0] = 3;
                bytes
            }
            ActuatorCommand::StopPercuss => {
                let mut bytes = [0u8; 5];
                bytes[0] = 4;
                bytes
            }
        }
    }

    /// Speed is a percentage of the max the actuator can go.
    pub fn set_speed(mut speed: f64, actuator: Actuator, direction: Direction) -> Self {
        speed = speed.clamp(0.0, 1.0);
        ActuatorCommand::SetSpeed((speed * u16::MAX as f64) as u16, actuator, direction)
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

impl ActuatorReading {
    pub fn serialize(&self) -> [u8; 4] {
        let mut bytes = [0, 0, 0, 0u8];
        bytes[0..=1].copy_from_slice(&self.m1_reading.to_le_bytes());
        bytes[2..=3].copy_from_slice(&self.m2_reading.to_le_bytes());
        bytes
    }
    pub fn deserialize(bytes: [u8; 4]) -> Self {
        // this expect is safe
        let m1_reading =
            u16::from_le_bytes(bytes[0..=1].try_into().expect("wrong number of bytes"));
        let m2_reading =
            u16::from_le_bytes(bytes[2..=3].try_into().expect("wrong number of bytes"));
        Self {
            m1_reading,
            m2_reading,
        }
    }
}

impl FromPicoV3 {
    /// 1 tag + 4 FromImu (4×25) + 1 ActuatorReading (4)  = 105 bytes
    pub const SIZE: usize = 105;

    pub fn serialize(&self) -> [u8; Self::SIZE] {
        let mut bytes = [0u8; Self::SIZE];

        match self {
            FromPicoV3::Reading(readings, act) => {
                bytes[0] = 0;
                for (i, r) in readings.iter().enumerate() {
                    let start = 1 + i * FromIMU::SIZE;
                    let end = start + FromIMU::SIZE;
                    bytes[start..end].copy_from_slice(&r.serialize());
                }
                bytes[Self::SIZE - 4..].copy_from_slice(&act.serialize());
            }
            FromPicoV3::Error => bytes[0] = 3,
        }
        bytes
    }

    pub fn deserialize(bytes: [u8; Self::SIZE]) -> Result<Self, &'static str> {
        match bytes[0] {
            0 => {
                let mut readings: [FromIMU; 4] = [FromIMU::Error; 4];
                for i in 0..4 {
                    let start = 1 + i * FromIMU::SIZE;
                    let end = start + FromIMU::SIZE;
                    let imu_bytes: [u8; FromIMU::SIZE] =
                        bytes[start..end].try_into().map_err(|_| "slice size")?;
                    readings[i] = FromIMU::deserialize(imu_bytes)?;
                }
                let act_bytes: [u8; 4] = bytes[Self::SIZE - 4..]
                    .try_into()
                    .map_err(|_| "act slice")?;

                let act = ActuatorReading::deserialize(act_bytes);
                Ok(FromPicoV3::Reading(readings, act))
            }
            3 => Ok(FromPicoV3::Error),
            _ => Err("invalid FromPicoV3 tag"),
        }
    }
}

#[cfg(test)]
#[cfg(feature = "std")]
mod tests {
    use super::*;

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
        let rate = AngularRate { x: 0.1, y: 0.2, z: 0.3 };
        let accel = AccelerationNorm { x: 0.0, y: -9.81, z: 0.0 };
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
    fn actuator_reading_roundtrip() {
        let original = ActuatorReading { m1_reading: 1234, m2_reading: 5678 };
        let bytes = original.serialize();
        let result = ActuatorReading::deserialize(bytes);
        assert_eq!(original, result);
    }

    #[test]
    fn actuator_command_set_speed_roundtrip() {
        for actuator in [Actuator::Lift, Actuator::Bucket, Actuator::Dumper] {
            for direction in [Direction::Forward, Direction::Backward] {
                let original = ActuatorCommand::SetSpeed(12345, actuator, direction);
                let bytes = original.serialize();
                let result = ActuatorCommand::deserialize(bytes).unwrap();
                assert_eq!(original, result);
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
        assert_eq!(start, ActuatorCommand::deserialize(start.serialize()).unwrap());

        let stop = ActuatorCommand::StopPercuss;
        assert_eq!(stop, ActuatorCommand::deserialize(stop.serialize()).unwrap());
    }

    #[test]
    fn actuator_command_set_speed_helper() {
        let cmd = ActuatorCommand::set_speed(0.5, Actuator::Lift, Direction::Forward);
        let bytes = cmd.serialize();
        let result = ActuatorCommand::deserialize(bytes).unwrap();
        assert_eq!(cmd, result);
    }

    #[test]
    fn from_pico_v3_reading_roundtrip() {
        let rate = AngularRate { x: 1.0, y: 2.0, z: 3.0 };
        let accel = AccelerationNorm { x: 0.0, y: -9.81, z: 0.0 };
        let readings = [
            FromIMU::Reading(rate, accel),
            FromIMU::NoDataReady,
            FromIMU::Error,
            FromIMU::Reading(
                AngularRate { x: -1.0, y: 0.0, z: 0.5 },
                AccelerationNorm { x: 1.0, y: 1.0, z: 1.0 },
            ),
        ];
        let act = ActuatorReading { m1_reading: 100, m2_reading: 200 };
        let original = FromPicoV3::Reading(readings, act);
        let bytes = original.serialize();
        let result = FromPicoV3::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    #[test]
    fn from_pico_v3_error_roundtrip() {
        let original = FromPicoV3::Error;
        let bytes = original.serialize();
        let result = FromPicoV3::deserialize(bytes).unwrap();
        assert_eq!(original, result);
    }

    #[test]
    fn actuator_command_invalid_tag() {
        let bytes = [255, 0, 0, 0, 0];
        assert!(ActuatorCommand::deserialize(bytes).is_err());
    }

    #[test]
    fn actuator_command_invalid_actuator() {
        let mut bytes = ActuatorCommand::SetSpeed(100, Actuator::Lift, Direction::Forward).serialize();
        bytes[3] = 255;
        assert!(ActuatorCommand::deserialize(bytes).is_err());
    }

    #[test]
    fn from_imu_invalid_tag() {
        let mut bytes = [0u8; FromIMU::SIZE];
        bytes[0] = 255;
        assert!(FromIMU::deserialize(bytes).is_err());
    }

    #[test]
    fn from_pico_v3_invalid_tag() {
        let mut bytes = [0u8; FromPicoV3::SIZE];
        bytes[0] = 255;
        assert!(FromPicoV3::deserialize(bytes).is_err());
    }
}
