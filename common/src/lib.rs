#![feature(f16)]
use bytemuck::{Pod, Zeroable};
use nalgebra::{Isometry3, Vector2, Vector3};
use crossbeam::atomic::AtomicCell;
use std::{sync::Arc, time::Duration};
use once_cell::sync::Lazy;

#[repr(C)]
#[derive(bincode::Encode, bincode::Decode,bitcode::Encode, bitcode::Decode, Clone, Copy, PartialEq, Eq, Pod, Zeroable)]
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


use std::io::Write;

use bitcode::{Decode, Encode};
use embedded_common::{Actuator, ActuatorCommand};
use nalgebra::{distance, Point2, Point3};

// Taken from https://opus-codec.org/docs/opus_api-1.5/group__opus__encoder.html#gad2d6bf6a9ffb6674879d7605ed073e25
pub const AUDIO_FRAME_SIZE: u32 = 960;
pub const AUDIO_SAMPLE_RATE: u32 = 48000;
pub const THALASSIC_CELL_SIZE: f32 = 0.03125;
pub const THALASSIC_WIDTH: u32 = 160;
pub const THALASSIC_HEIGHT: u32 = 224;
pub const THALASSIC_CELL_COUNT: u32 = THALASSIC_WIDTH * THALASSIC_HEIGHT;

/// cells don't have a y value but world points do, so please provide one one
pub fn cell_to_world_point((x, z): (usize, usize), y: f64) -> Point3<f64> {
    Point3::new(
        x as f64 * THALASSIC_CELL_SIZE as f64,
        y,
        z as f64 * THALASSIC_CELL_SIZE as f64,
    )
}
pub fn world_point_to_cell(point: Point3<f64>) -> (usize, usize) {
    (
        (point.x / THALASSIC_CELL_SIZE as f64) as usize,
        (point.z / THALASSIC_CELL_SIZE as f64) as usize,
    )
}

#[derive(Debug, Clone, Copy)]
/// a rectangular area on the thalassic cell map
///
/// larger x = further left, so `left` should have a larger numeric value than `right`
pub struct CellsRect {
    pub top: usize,
    pub bottom: usize,
    pub left: usize,
    pub right: usize,
}
impl CellsRect {
    pub fn new((left, bottom): (f64, f64), width_meters: f64, height_meters: f64) -> Self {
        let left = left / THALASSIC_CELL_SIZE as f64;
        let bottom = bottom / THALASSIC_CELL_SIZE as f64;

        Self {
            left: left.round() as usize,
            bottom: bottom.round() as usize,
            right: (left - (width_meters / THALASSIC_CELL_SIZE as f64)).round() as usize,
            top: (bottom + (height_meters / THALASSIC_CELL_SIZE as f64)).round() as usize,
        }
    }

    /// ensure this rect is at least `padding` cells away from each world border
    pub fn pad_from_world_border(&self, padding: usize) -> Self {
        Self {
            top: self.top.min(THALASSIC_HEIGHT as usize - padding),
            bottom: self.bottom.max(padding),
            left: self.left.min(THALASSIC_WIDTH as usize - padding),
            right: self.right.max(padding),
        }
    }
}


#[repr(u8)]
#[derive(Debug, bitcode::Encode, bitcode::Decode, Clone, Copy, PartialEq, Eq, bincode::Encode, bincode::Decode)]
pub enum LunabotStage {
    TeleOp = 0,
    SoftStop = 1,
    Autonomy = 2,
}

impl TryFrom<u8> for LunabotStage {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::TeleOp),
            1 => Ok(Self::SoftStop),
            2 => Ok(Self::Autonomy),
            _ => Err(())
        }
    }
}

#[derive(bincode::Encode, bincode::Decode, Debug, Encode, Decode, Clone, Copy, PartialEq)]
pub enum FromLunabase {
    Pong,
    ContinueMission,
    Steering(Steering),
    LiftActuators(i8),
    BucketActuators(i8),
    LiftShake,
    Navigate((f32, f32)),
    DigDump((f32, f32)),
    SoftStop,
    StartPercuss,
    StopPercuss,
}

impl ToString for FromLunabase {
    fn to_string(&self) -> String {
        format!("{self:?}")
    }
}

impl ToString for FromAI {
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

#[derive(Debug, bitcode::Encode, bitcode::Decode, Clone, Copy, bincode::Encode, bincode::Decode)]
pub enum FromLunabot {
    RobotIsometry { origin: [f32; 3], quat: [f32; 4] },
    ArmAngles {
        hinge: f32,
        bucket: f32
    },
    Ping(LunabotStage),
}

impl FromLunabot {
    fn write_code(&self, mut w: impl Write) -> std::io::Result<()> {
        let bytes = bitcode::encode(self);
        write!(w, "{self:?} = 0x")?;
        for b in bytes {
            write!(w, "{b:x}")?;
        }
        writeln!(w, "")
    }

    pub fn write_code_sheet(mut w: impl Write) -> std::io::Result<()> {
        FromLunabot::Ping(LunabotStage::TeleOp).write_code(&mut w)?;
        FromLunabot::Ping(LunabotStage::SoftStop).write_code(&mut w)?;
        FromLunabot::Ping(LunabotStage::Autonomy).write_code(&mut w)?;
        Ok(())
    }
}


#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathKind {
    MoveOntoTarget,
    StopInFrontOfTarget,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathInstruction {
    MoveTo,
    FaceTowards,
    MoveToBackwards,
}

#[derive(Debug, Clone, Copy)]
pub struct PathPoint {
    pub cell: (usize, usize),
    pub instruction: PathInstruction,
}
impl PathPoint {
    /// min distance for robot to be considered at a point
    const AT_POINT_THRESHOLD: f64 = 0.2;

    /// min radians gap between robot  for robot to be considered facing towards a point
    const FACING_TOWARDS_THRESHOLD: f64 = 0.2;

    pub fn is_finished(&self, robot_pos: &Point2<f64>, robot_heading: &Point2<f64>) -> bool {
        let world_pos = cell_to_world_point(self.cell, 0.).xz();

        match self.instruction {
            PathInstruction::MoveTo | PathInstruction::MoveToBackwards => {
                distance(&world_pos, robot_pos) < Self::AT_POINT_THRESHOLD
            }

            PathInstruction::FaceTowards => {
                (world_pos - robot_pos).angle(&robot_heading.coords)
                    < Self::FACING_TOWARDS_THRESHOLD
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Ellipse {
    h: f64,
    k: f64,
    radius_x: f64,
    radius_y: f64,
}

/// units are in cells
#[derive(Debug, Clone)]
pub enum Obstacle {
    Rect(CellsRect),
    Ellipse(Ellipse),
}
impl Obstacle {
    /// width and height must be positive
    pub fn new_rect(left_bottom: (f64, f64), width_meters: f64, height_meters: f64) -> Obstacle {
        Obstacle::Rect(CellsRect::new(left_bottom, width_meters, height_meters))
    }

    pub fn new_ellipse(center: (f64, f64), radius_x_meters: f64, radius_y_meters: f64) -> Obstacle {
        Obstacle::Ellipse(Ellipse {
            h: center.0 / THALASSIC_CELL_SIZE as f64,
            k: center.1 / THALASSIC_CELL_SIZE as f64,
            radius_x: radius_x_meters / THALASSIC_CELL_SIZE as f64,
            radius_y: radius_y_meters / THALASSIC_CELL_SIZE as f64,
        })
    }

    pub fn new_circle(center: (f64, f64), radius_meters: f64) -> Obstacle {
        Self::new_ellipse(center, radius_meters, radius_meters)
    }

    pub fn contains_cell(&self, (x, y): (usize, usize)) -> bool {
        match self {
            Obstacle::Rect(CellsRect {
                top,
                bottom,
                left,
                right,
            }) => {
                *right <= x && x <= *left && *bottom <= y && y <= *top // larger x = further left
            }
            Obstacle::Ellipse(Ellipse {
                h,
                k,
                radius_x,
                radius_y,
            }) => {
                (((x as f64 - h) * (x as f64 - h)) / (radius_x * radius_x))
                    + (((y as f64 - k) * (y as f64 - k)) / (radius_y * radius_y))
                    <= 1.0
            }
        }
    }
}

pub const AI_HEARTBEAT_RATE: Duration = Duration::from_millis(50);
pub const HOST_HEARTBEAT_LISTEN_RATE: Duration = Duration::from_millis(500);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ParseError {
    NotEnoughBytes {
        bytes_needed: usize
    },
    InvalidData
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, Pod, Zeroable, PartialEq, Eq)]
pub struct Occupancy(u32);

impl Occupancy {
    pub const UNKNOWN: Self = Self(0);
    pub const FREE: Self = Self(1);
    pub const OCCUPIED: Self = Self(2);

    pub fn occupied(self) -> bool {
        // True iff the cell is not empty
        self.0 == 2
    }
}


#[repr(u8)]
enum FromHostHeader {
    BaseIsometry = 0,
    FromLunabase = 1,
    ActuatorReadings = 2,
    ThalassicData = 3
}

#[derive(Debug)]
pub enum FromHost {
    BaseIsometry {
        isometry: Isometry3<f64>
    },
    FromLunabase {
        msg: FromLunabase
    },
    ActuatorReadings {
        lift: u16,
        bucket: u16
    },
    ThalassicData {
        obstacle_map: Box<[Occupancy; THALASSIC_CELL_COUNT as usize]>
    }
}



#[derive(Debug, Clone, PartialEq)]
pub enum FromAI {
    SetSteering(Steering),
    SetActuators(ActuatorCommand),
    Heartbeat,
    StartPercuss,
    StopPercuss,
    SetStage(LunabotStage),
    RequestThalassic,
    PathFound(Box<Vec<Vector2<f64>>>)
}

impl bincode::Encode for FromAI {
    fn encode<E: bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), bincode::error::EncodeError> {
        use bincode::Encode;
        match self {
            FromAI::SetSteering(steering) => {
                0u8.encode(encoder)?;
                steering.encode(encoder)?;
            }
            FromAI::SetActuators(cmd) => {
                1u8.encode(encoder)?;
                let bytes: [u8; 5] = cmd.serialize();
                bytes.encode(encoder)?;
            }
            FromAI::Heartbeat => {
                2u8.encode(encoder)?;
            }
            FromAI::StartPercuss => {
                3u8.encode(encoder)?;
            }
            FromAI::StopPercuss => {
                4u8.encode(encoder)?;
            }
            FromAI::SetStage(stage) => {
                5u8.encode(encoder)?;
                (*stage as u8).encode(encoder)?;
            }
            FromAI::RequestThalassic => {
                6u8.encode(encoder)?;
            }
            FromAI::PathFound(path) => {
                7u8.encode(encoder)?;
                (path.len() as u64).encode(encoder)?;
                for point in path.iter() {
                    point.x.encode(encoder)?;
                    point.y.encode(encoder)?;
                }
            }
        }
        Ok(())
    }
}

impl bincode::Decode<()> for FromAI {
    fn decode<D: bincode::de::Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, bincode::error::DecodeError> {
        use bincode::Decode;
        let variant: u8 = Decode::decode(decoder)?;
        match variant {
            0 => {
                let steering: Steering = Decode::decode(decoder)?;
                Ok(FromAI::SetSteering(steering))
            }
            1 => {
                let bytes: [u8; 5] = Decode::decode(decoder)?;
                let cmd = ActuatorCommand::deserialize(bytes)
                    .map_err(|e| bincode::error::DecodeError::OtherString(e.into()))?;
                Ok(FromAI::SetActuators(cmd))
            }
            2 => Ok(FromAI::Heartbeat),
            3 => Ok(FromAI::StartPercuss),
            4 => Ok(FromAI::StopPercuss),
            5 => {
                let stage_val: u8 = Decode::decode(decoder)?;
                let stage = LunabotStage::try_from(stage_val)
                    .map_err(|_| bincode::error::DecodeError::OtherString("Invalid stage value".into()))?;
                Ok(FromAI::SetStage(stage))
            }
            6 => Ok(FromAI::RequestThalassic),
            7 => {
                let len: u64 = Decode::decode(decoder)?;
                let mut vec = Vec::with_capacity(len as usize);
                for _ in 0..len {
                    let x: f64 = Decode::decode(decoder)?;
                    let y: f64 = Decode::decode(decoder)?;
                    vec.push(Vector2::new(x, y));
                }
                Ok(FromAI::PathFound(Box::new(vec)))
            }
            _ => Err(bincode::error::DecodeError::OtherString("Invalid variant".into())),
        }
    }
}

impl bincode::Encode for FromHost {
    fn encode<E: bincode::enc::Encoder>(&self, encoder: &mut E) -> Result<(), bincode::error::EncodeError> {
        use bincode::Encode;
        match self {
            FromHost::BaseIsometry { isometry } => {
                0u8.encode(encoder)?;
                // encode translation (x,y,z) and quaternion (w,x,y,z)
                let translation = isometry.translation.vector;
                translation.x.encode(encoder)?;
                translation.y.encode(encoder)?;
                translation.z.encode(encoder)?;
                let q = isometry.rotation;
                q.w.encode(encoder)?;
                q.i.encode(encoder)?;
                q.j.encode(encoder)?;
                q.k.encode(encoder)?;
            }
            FromHost::FromLunabase { msg } => {
                1u8.encode(encoder)?;
                msg.encode(encoder)?;
            }
            FromHost::ActuatorReadings { lift, bucket } => {
                2u8.encode(encoder)?;
                (*lift).encode(encoder)?;
                (*bucket).encode(encoder)?;
            }
            FromHost::ThalassicData { obstacle_map } => {
                3u8.encode(encoder)?;
                // encode as raw bytes of occupancy values
                for occ in obstacle_map.iter() {
                    occ.0.encode(encoder)?;
                }
            }
        }
        Ok(())
    }
}

impl bincode::Decode<()> for FromHost {
    fn decode<D: bincode::de::Decoder<Context = ()>>(decoder: &mut D) -> Result<Self, bincode::error::DecodeError> {
        use bincode::Decode;
        let variant: u8 = Decode::decode(decoder)?;
        match variant {
            0 => {
                let tx: f64 = Decode::decode(decoder)?;
                let ty: f64 = Decode::decode(decoder)?;
                let tz: f64 = Decode::decode(decoder)?;
                let qw: f64 = Decode::decode(decoder)?;
                let qi: f64 = Decode::decode(decoder)?;
                let qj: f64 = Decode::decode(decoder)?;
                let qk: f64 = Decode::decode(decoder)?;
                let iso = Isometry3::from_parts(
                    nalgebra::Translation3::new(tx, ty, tz),
                    nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(qw, qi, qj, qk)),
                );
                Ok(FromHost::BaseIsometry { isometry: iso })
            }
            1 => {
                let msg: FromLunabase = Decode::decode(decoder)?;
                Ok(FromHost::FromLunabase { msg })
            }
            2 => {
                let lift: u16 = Decode::decode(decoder)?;
                let bucket: u16 = Decode::decode(decoder)?;
                Ok(FromHost::ActuatorReadings { lift, bucket })
            }
            3 => {
                let mut map: Box<[Occupancy; THALASSIC_CELL_COUNT as usize]> = Box::new([Occupancy::UNKNOWN; THALASSIC_CELL_COUNT as usize]);
                for occ in map.iter_mut() {
                    let val: u32 = Decode::decode(decoder)?;
                    occ.0 = val;
                }
                Ok(FromHost::ThalassicData { obstacle_map: map })
            }
            _ => Err(bincode::error::DecodeError::OtherString("Invalid variant".into())),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bincode::{encode_to_vec, decode_from_slice};

    fn roundtrip(value: &FromAI) {
        let config = bincode::config::standard();
        let bytes = encode_to_vec(value, config).expect("encode");
        let (decoded, _): (FromAI, usize) = decode_from_slice::<FromAI, _>(&bytes, config).expect("decode");
        assert_eq!(*value, decoded);
    }

    #[test]
    fn test_set_steering() {
        let value = FromAI::SetSteering(Steering::new(0.5, -0.7, 3.14));
        roundtrip(&value);
    }

    #[test]
    fn test_set_actuators() {
        let value = FromAI::SetActuators(ActuatorCommand::Shake);
        roundtrip(&value);
    }

    #[test]
    fn test_heartbeat() {
        roundtrip(&FromAI::Heartbeat);
    }

    #[test]
    fn test_percuss() {
        roundtrip(&FromAI::StartPercuss);
        roundtrip(&FromAI::StopPercuss);
    }

    #[test]
    fn test_set_stage() {
        roundtrip(&FromAI::SetStage(LunabotStage::Autonomy));
    }

    #[test]
    fn test_request_thalassic() {
        roundtrip(&FromAI::RequestThalassic);
    }

    #[test]
    fn test_path_found() {
        let vec_points = vec![Vector2::new(1.0, 2.0), Vector2::new(-3.5, 4.2)];
        roundtrip(&FromAI::PathFound(Box::new(vec_points)));
    }

    #[test]
    fn test_invalid_variant() {
        let config = bincode::config::standard();
        // Variant 255 with no data
        let bytes = vec![255u8];
        let err = decode_from_slice::<FromAI, _>(&bytes, config).unwrap_err();
        match err {
            bincode::error::DecodeError::OtherString(_) => { /* expected */ }
            _ => panic!("Unexpected error variant"),
        }
    }
}

// Add global shared Lunabot stage atomic cell to synchronize stage information across components.
pub static LUNABOT_STAGE: Lazy<Arc<AtomicCell<LunabotStage>>> = Lazy::new(|| Arc::new(AtomicCell::new(LunabotStage::SoftStop)));