use realsense_rust::{
    context::Context,
    kind::{Rs2CameraInfo, Rs2ProductLine},
};
use std::collections::HashSet;

fn main() {
    let context = Context::new().expect("Failed to create RealSense context");

    let mut product_mask = HashSet::new();
    product_mask.insert(Rs2ProductLine::Depth);
    // product_mask.insert(Rs2ProductLine::T200);

    let devices = context.query_devices(product_mask);

    // let device = devices
    //     .into_iter()
    //     .next()
    //     .expect("No RealSense depth device found");
    // while let Some(device) = devices.iter().next_chunk()

    for device in devices {
        let name = device
            .info(Rs2CameraInfo::Name)
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "Unknown".to_string());

        println!("Resetting device: {name}");

        device.hardware_reset();

        println!("Hardware reset triggered.");
    }
}
