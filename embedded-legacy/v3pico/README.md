### Usage

1. Flashing Firmware
    * First set the PICO_SERIAL env variable to a number representing what you want the picos serial number to be.
    * ensure debug probe is attached
    * execute cargo run

2. Interacting with the pico
    * Each pico will create a new device /dev/ttyACM* with the serial number set to whatever the environment variable was.


### Features

1. Actuator control
2. Actuator length reading


### Pre-reqs

1. install probe-rs
2. flip link linker ```cargo install flip-link```
3. add the thumbv6m-none-eabi toolchain


### Udev Rules for resets:

# /etc/udev/rules.d/99-usb-reset.rules
# Allow ioctl-based resets on the C0DE:CAFE device
SUBSYSTEM=="usb", ATTR{idVendor}=="c0de", ATTR{idProduct}=="cafe", MODE="0666"


### Trouble Shooting
Occasionally the pico becomes unresponsive and the only way to fix it is unplug and replug. I have no idea why this is.


### Installing reset tool
1. run ```cargo make usbreset```
2. copy that binary to /usr/local/bin