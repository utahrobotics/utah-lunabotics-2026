## Setup
1. Install [probe-rs](https://probe.rs)
2. Attach debug probe to the rpi pico.

## Usage
1. Set the ACTUATOR_SERIAL environment variable to anything, because that will be used to set the serial number of the pico when it is pluged in via micro usb so that the computer can detect that it is there.
2. ```cargo run``` will use probe-rs to flash the firmware to the pico.
3. The pico will wait for a connection via microusb.
4. You should be able to control the actuator using [this.](https://github.com/matthewashton-k/actuator-controller)

All logs are sent over the probe and are displayed by probe-run when you call cargo run.

Lift actuator length in meters is sent over tty every 500 ms as a little endian encoded f64.

to control the bucket or the lift send a serialized ActuatorCommand from embedded common over tty.
