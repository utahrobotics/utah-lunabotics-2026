mod camera_info;
mod constants;
mod depth_camera;
mod enumerate;
mod iceoryx_utils;
mod thalassic;

use camera_info::DepthCameraInfo;
use enumerate::enumerate_depth_cameras;
use std::time::Duration;

use crate::constants::REALSENSE_SERIAL;

fn main() {
    println!("Starting RealSense depth camera publisher with occupancy grid");

    let cameras = vec![DepthCameraInfo {
        serial: REALSENSE_SERIAL.to_string(),
        depth_enabled: true,
        occupancy_enabled: true,
    }];

    enumerate_depth_cameras(cameras);

    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}
