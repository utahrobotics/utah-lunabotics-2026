# Guide

This document demonstrates how to use the crate, and documents how the VESC protocol works.

## VESC Protocol

All VESC messages carry two key pieces of information: the command ID, and the command payload data.

The command ID is just a one byte, big endian, unsigned integer assigned to each different command that can be sent. For example, SetDutyCycle is command 0 and SetRpm is command 3. See the VESC firmware documentation for more info on specific commands (<https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md>).

The payload data is just a 32 bit, big endian, signed integer. Now, turning data directly into this integer isn’t always helpful. For example, duty cycles range from -1 to 1, which would leave exactly three duty cycles to pick form. To combat this problem, all VESC commands have an associated scaling that’s expected to be used. So, for example, duty cycle numbers are multiplied by 100,000 before being sent. This crate does this conversion automatically, taking in an f32 which will be scaled and converted to the proper format.

### CAN Bus Messages

*Note: Much of this information comes from the VESC firmware documentation, which you can find here: (<https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md>)*

On the CAN bus, there are several devices on one bus, and so an identifier is used to specify which device is being commanded. VESC device IDs are one byte, unsigned, big endian integers. VESC uses a normal extended ID CAN message format, which you can read about here (<https://en.wikipedia.org/wiki/CAN_bus#Extended_frame_format>).

All CAN messages have two parts where data can go: the ID, and the data region (which I will call the payload, for clarity).

The first of these, the ID section, is 29 bits long in the extended frame format that VESC uses. Only the last two bytes of this are used by the protocol. The first byte has the command ID, and the second byte has the device ID.

The second portion, the data payload section, just contains the data payload bits, as defined in the pervious section. In accordance with the CAN protocol, this can be trimmed to fewer bits if it has many leading zeros.

### UART (Direct) Messages

*Note: Much of this information comes from reverse engineering the output of the pyvesc package, some of which was done using the excellent CRCRevEng tool (<https://reveng.sourceforge.io/>).*

UART VESC messages follow a much stranger format. Here is a table showing it:

| **Byte** | 1-2 | 3 | 4-7 | 8-9 | 10 |
|----|----|----|----|----|----|
| **Value** | 0x0205 | 0x05 + Command ID | Payload | CRC of all previous bytes | 0x03 |

For some reason, there are a mess of bytes (0x020505) at the start, which the command ID is just added to. The CRC (cyclic redundancy check) is just a type of checksum. The parameters used here are as follows:

`width=16  poly=0x1021  init=0x0205  refin=false  refout=false  xorout=0x0000  check=0xbf1a  residue=0x0000`

Note that the command ID and payload data are both big-endian, as perviously mentioned.

## Crate Usage

### Base Functionality: `messages` Module

The messages module has a simple workflow:

* Create a `Message` with the data you want to send and its metadata.
* Use that `Message`’s methods to get whatever output format you need.

That workflow looks like this:

```rust
use vesc-translator::*;

fn main() {
  let my_direct_msg: Message = Message::new_no_target(CommandType::SetRpm, 1234.5);
  let my_message_bytes: Vec<u8> = &my_direct_msg.to_uart_binary();

  let target_id: u8 = 12_u8

  let my_can_msg: Message = Message::new(CommandType::SetRpm, target_id, 1234.5);
  let my_message_body_bytes: Vec<u8> = &my_can_msg.to_body_binary();
  let my_message_header_bytes: Vec<u8> = &my_can_msg.to_header_binary();
}
```

Note that if a message is created with no target id, then running `to_header_binary()` will panic, because the header must have an id. Also, the values given for the payload data when creating a message will be scaled according to the command being sent.

All the traits that `Message` implements are also public, so that you can create your own message formats.

### Fancy Functionality: `comms` Module

The `comms` module attempts to provide a seamless experience for controlling motors, and integrates communication with them hence the name. Currently, `comms` only supports communication with motors over CAN, but may integrate UART control in the future.

The workflow using `comms` is simple:

* Create a `Motor` struct with the id of that motor
* Use the motor’s methods to send messages

In the future, the motor struct should also let you get data from the motor which it sends out regularly onto the CAN bus.

Here’s an example of using `comms`:

```javascript
use vesc-translator::*;

fn main() {
  let my_motor: Motor = Motor::new(12);

  my_motor.set_rpm(1234.5);
}
```


