#![feature(f16)]
use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};
mod constants;
pub mod ports;
pub use constants::*;

#[repr(C)]
#[derive(
    bincode::Encode,
    bincode::Decode,
    bitcode::Encode,
    bitcode::Decode,
    Clone,
    Copy,
    PartialEq,
    Eq,
    Pod,
    Zeroable,
    Serialize,
)]
pub struct Steering {
    left: i8,
    right: i8,
    weight: u16,
}

impl std::fmt::Debug for Steering {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let (left, right) = self.get_left_and_right();
        f.debug_struct("Steering")
            .field("left", &left)
            .field("right", &right)
            .finish()
    }
}

impl Steering {
    pub const DEFAULT_WEIGHT: f64 = 25.0;

    pub fn get_left_and_right(self) -> (f64, f64) {
        (
            if self.left < 0 {
                -(self.left as f64) / i8::MIN as f64
            } else {
                self.left as f64 / i8::MAX as f64
            },
            if self.right < 0 {
                -(self.right as f64) / i8::MIN as f64
            } else {
                self.right as f64 / i8::MAX as f64
            },
        )
    }

    pub fn get_weight(self) -> f64 {
        f16::from_bits(self.weight) as f64
    }

    /* pub fn set_weight(&mut self, weight: f64){
      let weight = weight as f16;
      let weight = weight.to_bits();
      self.weight = weight;

    } */

    pub fn new(mut left: f64, mut right: f64, weight: f64) -> Self {
        left = left.max(-1.0).min(1.0);
        right = right.max(-1.0).min(1.0);

        let left = if left < 0.0 {
            (-left * i8::MIN as f64) as i8
        } else {
            (left * i8::MAX as f64) as i8
        };
        let right = if right < 0.0 {
            (-right * i8::MIN as f64) as i8
        } else {
            (right * i8::MAX as f64) as i8
        };
        let weight = weight as f16;
        let weight = weight.to_bits();
        Self {
            left,
            right,
            weight,
        }
    }
}

impl Default for Steering {
    fn default() -> Self {
        Self::new(0.0, 0.0, Self::DEFAULT_WEIGHT)
    }
}

#[derive(Clone, Copy, Debug, Default, Encode, Decode)]
pub struct IMUReading {
    pub angular_velocity: [f64; 3],
    pub acceleration: [f64; 3],
}

use std::{collections::HashMap, io::Write};

use bitcode::{Decode, Encode};
use embedded_common::{Actuator, ActuatorCommand};

#[repr(u8)]
#[derive(
    Debug,
    bitcode::Encode,
    bitcode::Decode,
    Clone,
    Copy,
    PartialEq,
    Eq,
    bincode::Encode,
    bincode::Decode,
    Serialize,
)]
pub enum LunabotStage {
    Manual = 0,
    SoftStop = 1,
    Autonomy = 2,
}

impl TryFrom<u8> for LunabotStage {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::Manual),
            1 => Ok(Self::SoftStop),
            2 => Ok(Self::Autonomy),
            _ => Err(()),
        }
    }
}

#[derive(
    bincode::Encode, bincode::Decode, Debug, Encode, Decode, Clone, Copy, PartialEq, Serialize,
)]
pub enum FromLunabase {
    ContinueMission,
    /// Skid steer message, 1,1 is full speed forward, -1,-1 is full speed back
    Steering(Steering),
    /// Move lift actuators, positive up, negative down  
    LiftActuators(i8),
    BucketActuators(i8),

    #[deprecated]
    LiftShake,
    /// Start autonomous mode, starting navigating to the requested x and y values.
    Navigate((f32, f32)),
    DigDump((f32, f32)),

    /// Request software stop mode
    SoftStop,

    /// Start and stop percussor
    StartPercuss,
    StopPercuss,

    /// disconnect events are technically not from the lunabase, they are manually enqueued in the lunabase copper task
    /// when the last seen packet from the lunabase exceeds the timeout
    Disconnect,
}

impl ToString for FromLunabase {
    fn to_string(&self) -> String {
        format!("{self:?}")
    }
}

impl Default for FromLunabase {
    fn default() -> Self {
        Self::SoftStop
    }
}

impl FromLunabase {
    fn write_code(&self, mut w: impl Write) -> std::io::Result<()> {
        let bytes = bitcode::encode(self);
        write!(w, "{self:?} = 0x")?;
        for b in bytes {
            write!(w, "{b:x}")?;
        }
        writeln!(w, "")
    }

    pub fn write_code_sheet(mut w: impl Write) -> std::io::Result<()> {
        // FromLunabase::Pong.write_code(&mut w)?;
        FromLunabase::ContinueMission.write_code(&mut w)?;
        FromLunabase::Steering(Steering::default()).write_code(&mut w)?;
        FromLunabase::SoftStop.write_code(&mut w)?;
        Ok(())
    }

    pub fn lift_shake() -> Self {
        FromLunabase::LiftShake
    }

    pub fn set_lift_actuator(mut speed: f64) -> Self {
        speed = speed.clamp(-1.0, 1.0);
        let speed = if speed < 0.0 {
            (-speed * i8::MIN as f64) as i8
        } else {
            (speed * i8::MAX as f64) as i8
        };
        FromLunabase::LiftActuators(speed)
    }

    pub fn set_bucket_actuator(mut speed: f64) -> Self {
        speed = speed.clamp(-1.0, 1.0);
        let speed = if speed < 0.0 {
            (-speed * i8::MIN as f64) as i8
        } else {
            (speed * i8::MAX as f64) as i8
        };
        FromLunabase::BucketActuators(speed)
    }

    pub fn get_lift_actuator_commands(self) -> Option<[ActuatorCommand; 2]> {
        match self {
            FromLunabase::LiftActuators(value) => Some(if value < 0 {
                [
                    ActuatorCommand::backward(Actuator::Lift),
                    ActuatorCommand::set_speed(value as f64 / i8::MIN as f64, Actuator::Lift),
                ]
            } else {
                [
                    ActuatorCommand::forward(Actuator::Lift),
                    ActuatorCommand::set_speed(value as f64 / i8::MAX as f64, Actuator::Lift),
                ]
            }),
            _ => None,
        }
    }

    pub fn get_bucket_actuator_commands(self) -> Option<[ActuatorCommand; 2]> {
        match self {
            FromLunabase::BucketActuators(value) => Some(if value < 0 {
                [
                    ActuatorCommand::forward(Actuator::Bucket),
                    ActuatorCommand::set_speed(value as f64 / i8::MIN as f64, Actuator::Bucket),
                ]
            } else {
                [
                    ActuatorCommand::backward(Actuator::Bucket),
                    ActuatorCommand::set_speed(value as f64 / i8::MAX as f64, Actuator::Bucket),
                ]
            }),
            _ => None,
        }
    }
}

#[derive(
    Debug,
    bitcode::Encode,
    bitcode::Decode,
    Clone,
    bincode::Encode,
    bincode::Decode,
    Serialize,
    Deserialize,
)]
pub enum FromLunabot {
    /// Reports the robots pose
    RobotIsometry {
        origin: [f32; 3],
        quat: [f32; 4],
    },
    /// Angle in degrees of the hinge and bucket
    ArmAngles {
        hinge: f32,
        bucket: f32,
    },
    RobotMotion {
        velocity:[f32;3],
        acceleration: [f32;3],
    },
    ErroredTasks(HashMap<String, String>),
}

impl Default for FromLunabot {
    fn default() -> Self {
        Self::ErroredTasks(HashMap::new())
    }
}
