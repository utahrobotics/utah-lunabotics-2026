use core::ffi;
use std::{
    fs,
    io::Error,
    os::fd::{AsRawFd, IntoRawFd},
};

use udev::{Device, Enumerator};

enum TargetDevice {
    T265,
    RealsenseDepth,
    V3Pico,
    Vesc,
}

impl TargetDevice {
    fn serial(&self) -> &'static str {
        match self {
            TargetDevice::RealsenseDepth => "Depth",
            TargetDevice::V3Pico => "USR_V3PICO",
            TargetDevice::T265 => "Movidius",
            TargetDevice::Vesc => "STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304"
        }
    }
}

/// auto discovers usb devices with an idserial that contains USR_V3PICO and then calls ioctl to reset the device
fn main() {
    let args = std::env::args().collect::<Vec<String>>();
    if args.len() < 2 {
        panic!("need target reset param: t265 or v3pico or depth");
    }

    // technically the target vendor or serial
    let target_serial = match args[1].as_str() {
        "depth" => TargetDevice::RealsenseDepth,
        "v3pico" => TargetDevice::V3Pico,
        "t265" | "T265" => TargetDevice::T265,
        "vesc" => TargetDevice::Vesc,
        _ => {
            panic!("unknown arg. options: depth, v3pico, t265");
        }
    }
    .serial();
    const USBDEVFS_RESET: ffi::c_uint = 21780;
    let mut enumerator = Enumerator::new().expect("failed to create enumerator");
    for device in enumerator.scan_devices().expect("failed to scan devices") {
        for property in device.properties() {
            if (property.name() == "ID_SERIAL"
                || property.name() == "ID_VENDOR"
                || property.name() == "ID_MODEL_FROM_DATABASE")
                && property.value().to_string_lossy().contains(target_serial)
            {
                if device.devnode().is_none() {
                    continue;
                }
                println!("attempting to reset device");
                let fd = fs::OpenOptions::new()
                    .read(true)
                    .write(true)
                    .open(device.devnode().unwrap())
                    .expect("couldn't open device")
                    .into_raw_fd();
                unsafe {
                    let result = ioctl(fd, USBDEVFS_RESET, 0);
                    println!("ioctl result: {result}");
                    // close(fd);
                }
            }
        }
    }
}

unsafe extern "C" {
    fn ioctl(fd: ffi::c_int, o: ffi::c_uint, arg: ffi::c_int) -> ffi::c_int;
    // fn close(fd: ffi::c_int);
}
