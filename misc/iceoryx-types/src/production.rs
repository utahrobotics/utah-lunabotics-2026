#![cfg(any(feature = "production"))]

use bincode::{Decode, Encode};
use common::{THALASSIC_CELL_COUNT, THALASSIC_HEIGHT, THALASSIC_WIDTH};
use iceoryx2::prelude::ZeroCopySend;
use nalgebra::Point3;
use serde::Serialize;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Encode, Decode, ZeroCopySend, Serialize)]
#[type_name("PointXYZIR")]
pub struct PointXYZIR {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub time: f32,
    pub ring: u16,
}

#[repr(C)]
#[derive(Clone, Debug, ZeroCopySend, Encode, Decode, Serialize)]
#[type_name("IceoryxOccupancyGrid")]
pub struct IceoryxOccupancyGrid {
    pub width: u32,
    pub height: u32,
    #[serde(serialize_with = "<[_]>::serialize")]
    pub data: [u32; THALASSIC_CELL_COUNT as usize],
}

#[repr(C)]
#[derive(Clone, Debug, ZeroCopySend, Encode, Decode, Serialize)]
pub struct IceoryxDepthFrame<const SIZE: usize> {
    #[serde(serialize_with = "<[_]>::serialize")]
    pub depths: [u16; SIZE],
    pub depth_scale: f32,
    pub focal_len: (f32, f32),
}

impl<const SIZE: usize> Default for IceoryxDepthFrame<SIZE> {
    fn default() -> Self {
        Self {
            depths: [0; SIZE],
            depth_scale: 0.0,
            focal_len: (383.0, 383.0),
        }
    }
}

impl Default for IceoryxOccupancyGrid {
    fn default() -> Self {
        Self {
            width: THALASSIC_WIDTH,
            height: THALASSIC_HEIGHT,
            data: [0; THALASSIC_CELL_COUNT as usize],
        }
    }
}

/// Maximum number of points stored in the fixed-size point-cloud message.
/// Reduced from 130 000 → 20 000 to keep message size (and stack usage in Rust) reasonable.
pub const MAX_POINT_CLOUD_POINTS: usize = 10000;

#[repr(C)]
#[derive(Clone, Debug, ZeroCopySend, Encode, Decode, Serialize)]
#[type_name("IceoryxPointCloud")]
pub struct IceoryxPointCloud {
    pub is_last: bool,
    pub publish_count: u64,
    #[serde(serialize_with = "<[_]>::serialize")]
    pub points: [PointXYZIR; MAX_POINT_CLOUD_POINTS],
}

impl Default for IceoryxPointCloud {
    fn default() -> Self {
        Self {
            is_last: true,
            publish_count: 0,
            points: [PointXYZIR::default(); MAX_POINT_CLOUD_POINTS],
        }
    }
}

#[derive(Debug)]
pub struct PointCloudAccumulator {
    points: Vec<PointXYZIR>,
}

impl Default for PointCloudAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl PointCloudAccumulator {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// returns Ok(Some(&points)) if this was the last message and point cloud is complete
    /// returns Ok(None) if more messages are expected
    /// returns Err if there's a publish count mismatch
    pub fn add_message(
        &mut self,
        cloud: &IceoryxPointCloud,
    ) -> Result<Option<&[PointXYZIR]>, &'static str> {
        self.points.extend_from_slice(&cloud.points);

        if cloud.is_last {
            Ok(Some(&self.points))
        } else {
            Ok(None)
        }
    }

    pub fn take_completed(&mut self) -> Vec<PointXYZIR> {
        let points = std::mem::take(&mut self.points);
        points
    }

    pub fn point_count(&self) -> usize {
        self.points.len()
    }

    pub fn reset(&mut self) {
        self.points.clear();
    }

    pub fn accumulated_points(&self) -> &[PointXYZIR] {
        &self.points
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Encode, Decode, ZeroCopySend, Serialize)]
#[type_name("ImuMsg")]
pub struct ImuMsg {
    pub quaternion: [f32; 4],
    pub angular_velocity: [f32; 3],
    pub linear_acceleration: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Encode, Decode, ZeroCopySend, Serialize)]
#[type_name("PoseMsg")]
pub struct PoseMsg {
    /// position in meters
    pub position: [f32; 3],
    /// orientation quaternion (w, x, y, z)
    pub quaternion: [f32; 4],
}

// -----------------------------------------------------------------------------
// AI ↔ Host byte-buffer messages
// -----------------------------------------------------------------------------

/// Maximum encoded size (in bytes) for a [`FromHost`](common::FromHost) message.
pub const FROM_HOST_MAX_BYTES: usize = 4096;

/// Maximum encoded size (in bytes) for a [`FromAI`](common::FromAI) message.
pub const FROM_AI_MAX_BYTES: usize = 4096;

/// Raw byte buffer transmitted from the host (robot) to the AI module.
///
/// The actual payload is stored in the first `len` bytes of `data` and is
/// encoded with `bincode`.
#[repr(C)]
#[derive(Clone, Copy, Debug, ZeroCopySend)]
#[type_name("FromHostBytes")]
pub struct FromHostBytes {
    pub len: u32,
    pub data: [u8; FROM_HOST_MAX_BYTES],
}

impl Default for FromHostBytes {
    fn default() -> Self {
        Self {
            len: 0,
            data: [0; FROM_HOST_MAX_BYTES],
        }
    }
}

/// Raw byte buffer transmitted from the AI module back to the host (robot).
#[repr(C)]
#[derive(Clone, Copy, Debug, ZeroCopySend)]
#[type_name("FromAIBytes")]
pub struct FromAIBytes {
    pub len: u32,
    pub data: [u8; FROM_AI_MAX_BYTES],
}

impl Default for FromAIBytes {
    fn default() -> Self {
        Self {
            len: 0,
            data: [0; FROM_AI_MAX_BYTES],
        }
    }
}

impl PointXYZIR {
    pub fn from_nalgebra(point: Point3<f64>, intensity: f32, time: f32, ring: u16) -> Self {
        Self {
            x: point.x as f32,
            y: point.y as f32,
            z: point.z as f32,
            intensity,
            time,
            ring,
        }
    }
    pub fn to_nalgebra(&self) -> Point3<f64> {
        Point3::new(self.x as f64, self.y as f64, self.z as f64)
    }
}
