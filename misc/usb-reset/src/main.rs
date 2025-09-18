use core::ffi;
use std::{
    fs,
    io::Error,
    os::fd::{AsRawFd, IntoRawFd},
};

use udev::{Device, Enumerator};

/// auto discovers usb devices with an idserial that contains USR_V3PICO and then calls ioctl to reset the device
fn main() {
    const USBDEVFS_RESET: ffi::c_uint = 21780;
    let mut enumerator = Enumerator::new().expect("failed to create enumerator");
    for device in enumerator.scan_devices().expect("failed to scan devices") {
        for property in device.properties() {
            if property.name() == "ID_SERIAL"
                && property.value().to_string_lossy().contains("USR_V3PICO")
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
