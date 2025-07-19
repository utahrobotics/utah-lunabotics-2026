// Copyright (c) 2024
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Common point types shared between C++ and Rust for iceoryx2 publish-subscribe.
//!
//! The struct layout **must** match the C++ definition used by the Unilidar
//! publisher (see `unilidar_iceoryx_publisher/include/imu_point_types.hpp`).
//! Keeping the same memory layout allows zero-copy transfer between the
//! languages.

use bincode::{Decode, Encode};
use serde::Serialize;
use iceoryx2::prelude::ZeroCopySend;

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

/// Maximum number of points stored in the fixed-size point-cloud message.
/// Reduced from 130 000 → 20 000 to keep message size (and stack usage in Rust) reasonable.
pub const MAX_POINT_CLOUD_POINTS: usize = 6000;

#[repr(C)]
#[derive(Clone, Debug, ZeroCopySend, Encode, Decode, Serialize)]
#[type_name("IceoryxPointCloud")]
pub struct IceoryxPointCloud {
    pub publish_count: u64,
    #[serde(serialize_with = "<[_]>::serialize")]
    pub points: [PointXYZIR; MAX_POINT_CLOUD_POINTS],
}


impl Default for IceoryxPointCloud {
    fn default() -> Self {
        Self {
            publish_count: 0,
            points: [PointXYZIR::default(); MAX_POINT_CLOUD_POINTS],
        }
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