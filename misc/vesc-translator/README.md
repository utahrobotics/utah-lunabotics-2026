# VESC Translator

*A simple solution for creating VESC messages in Rust.*

## Goals

This project is designed to create messages to send to VESC motor controllers. It is designed mainly to generate UART format messages for direct control of motor controllers, but the `comms` feature allows it to generate and send CAN messages over serial port on Linux systems. It was made for Utah Student Robotics at the University of Utah. See GUIDE.md for information about VESC and how to use this crate.

## Development

The project is currently in active development, and is not ready to use. It needs a table of messages it can send, proper tests, and more features.
Planned features include:
* Getting motor data from motor objects
* More generic message type format

### Credits

* Thank you to the author of this webpage (<https://dongilc.gitbook.io/openrobot-inc/tutorials/control-with-can]>) from which lots of information about VESC CAN control was used.
* Thank you to the creators of pyvesc (<https://github.com/LiamBindle/PyVESC>), which severed as a great example, and provided lots of information about VESC.
* Thank you to the contributors of CRCRevEng (<https://reveng.sourceforge.io/>) which I used to determine the CRC for UART control, and Ted Yapo for writing an article about this (<https://hackaday.com/2019/06/27/reverse-engineering-cyclic-redundancy-codes/>).
* Thank you to the contributors to the VESC protocol, documentation of VESC over CAN can be found here (<https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md>).

### Authors

* Hale Barber


