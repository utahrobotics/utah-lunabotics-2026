use core::ffi;
use std::{
    fs,
    io::Error,
    os::fd::{AsRawFd, IntoRawFd},
};

use udev::{Device, Enumerator};

enum TargetDevice {
    Realsense,
    V3Pico,
}

impl TargetDevice {
    fn serial(&self) -> &'static str {
        match self {
            TargetDevice::Realsense => "Intel",
            TargetDevice::V3Pico => "USR_V3PICO",
        }
    }
}

/// auto discovers usb devices with an idserial that contains USR_V3PICO and then calls ioctl to reset the device
fn main() {
    let args = std::env::args().collect::<Vec<String>>();
    if args.len() < 2 {
        panic!("need target reset param: realsense or v3pico");
    }

    // technically the target vendor or serial
    let target_serial = match args[1].as_str() {
        "realsense" => TargetDevice::Realsense,
        "v3pico" => TargetDevice::V3Pico,
        _ => {
            panic!("unknown arg. options: realsense, v3pico");
        }
    }
    .serial();
    const USBDEVFS_RESET: ffi::c_uint = 21780;
    let mut enumerator = Enumerator::new().expect("failed to create enumerator");
    for device in enumerator.scan_devices().expect("failed to scan devices") {
        for property in device.properties() {
            if (property.name() == "ID_SERIAL" || property.name() == "ID_VENDOR")
                && property.value().to_string_lossy().contains(target_serial)
            {
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
