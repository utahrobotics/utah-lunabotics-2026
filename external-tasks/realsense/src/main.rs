mod constants;
mod depth_camera;
mod enumerate;
mod iceoryx_utils;

use enumerate::enumerate_depth_cameras;
use std::time::Duration;


fn main() {
    println!("Starting RealSense depth camera and tracking camera publisher");
    enumerate_depth_cameras();

    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}
