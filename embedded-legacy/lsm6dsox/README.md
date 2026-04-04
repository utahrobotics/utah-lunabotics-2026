# lsm6dsox

Platform-agnostic embedded-hal driver for the STMicroelectronics LSM6DSOX iNEMO inertial module.

Provided functionality is inspired by the C implementation from ST,
but tries to provide a higher level interface where possible.

To provide measurements the [accelerometer] traits and [measurements] crate are utilized.



## Resources

[Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsox.pdf)

[LSM6DSOX at st.com](https://www.st.com/en/mems-and-sensors/lsm6dsox.html)


For application hints please also refer to the
[application note](https://www.st.com/resource/en/application_note/an5272-lsm6dsox-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf)
provided by ST.

## Features

- [`Accelerometer`](https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html) trait implementation
- [`embedded-hal`](https://crates.io/crates/embedded-hal) I²C support
- Gyroscope
- Tap recognition
- Interrupts
- Further features may be added in the future

## Examples
```rust
use accelerometer::Accelerometer;
use lsm6dsox::*;

let mut lsm = lsm6dsox::Lsm6dsox::new(i2c, SlaveAddress::Low, delay);

lsm.setup()?;
lsm.set_accel_sample_rate(DataRate::Freq52Hz)?;
lsm.set_accel_scale(AccelerometerScale::Accel16g)?;
if let Ok(reading) = lsm.accel_norm() {
    println!("Acceleration: {:?}", reading);
}
```
## License

Open Logistics Foundation License\
Version 1.3, January 2023

See the LICENSE file in the top-level directory.

## Contact

Fraunhofer IML Embedded Rust Group - <embedded-rust@iml.fraunhofer.de>
