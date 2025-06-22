#![no_std]

use core::ops::Not;

pub const IMU_READING_DELAY_MS: u64 = 10;

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    Forward = 0,
    Backward = 1,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
/// Used to specify which actuator a command is meant for.
pub enum Actuator {
    /// the lift
    Lift = 0,
    /// the bucket
    Bucket = 1,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ActuatorCommand {
    SetSpeed(u16, Actuator),
    SetDirection(Direction, Actuator),
    Shake,
    StartPercuss,
    StopPercuss
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// adc readings
pub struct ActuatorReading {
    pub m1_reading: u16,
    pub m2_reading: u16,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FromIMU {
    Reading(AngularRate, AccelerationNorm),
    NoDataReady,
    Error,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FromPicoV3 {
    Reading([FromIMU; 4], ActuatorReading),
    Error
}

/// Radians per second
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    pub fn deserialize(bytes: [u8; 5]) -> Result<Self, &'static str> {
        let actuator = {
            if bytes[3] == Actuator::Lift as u8 {
                Actuator::Lift
            } else if bytes[3] == Actuator::Bucket as u8 {
                Actuator::Bucket
            } else if bytes[0] == 2 { // shake command
                Actuator::Lift           
            } else {
                return Err("Unknown actuator specifier (not m1 or m2)");
            }
        };
        match bytes[0] {
            tag if tag == 0 => {
                let speed = u16::from_le_bytes(
                    bytes[1..=2]
                        .try_into()
                        .map_err(|_| "Wrong number of bytes in actuator command")?,
                );
                Ok(ActuatorCommand::SetSpeed(speed, actuator))
            }
            tag if tag == 1 => {
                let dir = match bytes[1] {
                    0 => Direction::Forward,
                    1 => Direction::Backward,
                    _ => return Err("Invalid direction value"),
                };
                Ok(ActuatorCommand::SetDirection(dir, actuator))
            }
            tag if tag == 2 => {
                Ok(ActuatorCommand::Shake)
            } 
            tag if tag == 3 => {
                Ok(ActuatorCommand::StartPercuss)
            } 
            tag if tag == 4 => {
                Ok(ActuatorCommand::StopPercuss)
            } 
            _ => Err("Invalid variant tag"),
        }
    }

    pub fn serialize(&self) -> [u8; 5] {
        match self {
            ActuatorCommand::SetSpeed(speed, actuator) => {
                let mut bytes = [0u8; 5];
                bytes[0] = 0;
                bytes[1..=2].copy_from_slice(&speed.to_le_bytes());
                bytes[3] = *actuator as u8;
                bytes
            }
            ActuatorCommand::SetDirection(dir, actuator) => {
                let mut bytes = [0u8; 5];
                bytes[0] = 1;
                bytes[1] = *dir as u8;
                bytes[2] = 0;
                bytes[3] = *actuator as u8;
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

    pub fn set_speed(mut speed: f64, actuator: Actuator) -> Self {
        speed = speed.clamp(0.0, 1.0);
        ActuatorCommand::SetSpeed((speed * u16::MAX as f64) as u16, actuator)
    }

    pub fn forward(actuator: Actuator) -> Self {
        ActuatorCommand::SetDirection(Direction::Forward, actuator)
    }

    pub fn backward(actuator: Actuator) -> Self {
        ActuatorCommand::SetDirection(Direction::Backward, actuator)
    }
}

impl Not for Direction {
    type Output = Self;

    fn not(self) -> Self::Output {
        if self == Self::Forward {
            return Self::Backward
        } else {
            return Self::Forward
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
                    let end   = start + FromIMU::SIZE;
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
                    let end   = start + FromIMU::SIZE;
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
