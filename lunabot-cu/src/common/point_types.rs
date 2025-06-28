// Copyright (c) 2024
// SPDX-License-Identifier: Apache-2.0 OR MIT

//! Common point types shared between C++ and Rust for iceoryx2 publish-subscribe.
//!
//! The struct layout **must** match the C++ definition used by the Unilidar
//! publisher (see `unilidar_iceoryx_publisher/include/imu_point_types.hpp`).
//! Keeping the same memory layout allows zero-copy transfer between the
//! languages.

use bincode::{Decode, Encode};
use iceoryx2::prelude::ZeroCopySend;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Encode, Decode, ZeroCopySend)]
#[type_name("PointXYZIR")]
pub struct PointXYZIR {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: f32,
    pub time: f32,
    pub ring: u16,
}

pub const MAX_POINT_CLOUD_POINTS: usize = 60000;


#[repr(C)]
#[derive(Clone, Debug, ZeroCopySend)]
#[type_name("IceoryxPointCloud")]
pub struct IceoryxPointCloud {
    pub publish_count: u64,
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
#[derive(Clone, Copy, Debug, Default, Encode, Decode, ZeroCopySend)]
#[type_name("ImuMsg")]
pub struct ImuMsg {
    pub quaternion: [f32; 4],
    pub angular_velocity: [f32; 3],
    pub linear_acceleration: [f32; 3],
}