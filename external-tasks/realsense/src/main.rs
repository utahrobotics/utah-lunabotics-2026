mod constants;
mod depth_camera;
mod enumerate;
mod iceoryx_utils;

use enumerate::enumerate_depth_cameras;
use std::time::Duration;

use crate::constants::{REALSENSE_SERIAL, T265_SERIAL};

fn main() {
    println!("Starting RealSense depth camera publisher with occupancy grid");
    std::thread::sleep(Duration::from_secs(3)); // dont question it
    enumerate_depth_cameras(&[T265_SERIAL, REALSENSE_SERIAL]);

    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}
