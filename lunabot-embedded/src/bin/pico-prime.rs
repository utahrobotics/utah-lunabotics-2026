//! Template

#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    peripherals::USB,
    usb::{Driver, InterruptHandler},
};
use embassy_usb::{
    UsbDevice,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// something to read from usb
// something to forward to other pico
// something to read from sensors or other things

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

static MAX_MESSAGE_SIZE: usize = 256;
static PACKET_SIZE: u16 = 64;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("USR");
        config.product = Some("USR-PICO-PRIME");
        config.serial_number = Some("USR-PICO-PRIME");
        config.max_power = 100;
        config.max_packet_size_0 = 64;
        config
    };

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut builder = {
        static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
        static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

        let builder = embassy_usb::Builder::new(
            driver,
            config,
            CONFIG_DESCRIPTOR.init([0; 256]),
            BOS_DESCRIPTOR.init([0; 256]),
            &mut [], // no msos descriptors
            CONTROL_BUF.init([0; 64]),
        );
        builder
    };

    // Create classes on the builder.
    let class: CdcAcmClass<'_, Driver<'_, USB>> = {
        static STATE: StaticCell<State> = StaticCell::new();
        let state = STATE.init(State::new());
        CdcAcmClass::new(&mut builder, state, PACKET_SIZE)
    };

    let (class_tx, class_rx) = class.split();

    // Build the builder.
    let usb = builder.build();

    // Run the USB device.
    spawner.spawn(usb_task(usb)).unwrap();
    spawner.spawn(usb_tx_loop(class_tx)).unwrap();
    spawner.spawn(usb_rx_loop(class_rx)).unwrap();

    info!("Hello World!");
    loop {}
}

#[embassy_executor::task]
async fn usb_tx_loop(mut writer: Sender<'static, Driver<'static, USB>>) {
    // check if there is anything available from the secondary pico over UART (or maybe request information from the secondary pico?)
    // read (and decode?) the information off the secondary pico
    // send it over usb to the host
}

#[embassy_executor::task]
async fn usb_rx_loop(mut reader: Receiver<'static, Driver<'static, USB>>) {
    // probably some optimizations you can do here to calculate the max overhead we would need for COBS, making this buf smaller.
    let mut decoded_buf = [0u8; MAX_MESSAGE_SIZE * 2];
    let mut encoded_buf = [0u8; PACKET_SIZE as usize];
    let mut decoder = cobs::CobsDecoder::new(&mut decoded_buf);
    loop {
        reader.wait_connection().await;
        while let Ok(n) = reader.read_packet(&mut encoded_buf).await {
            match decoder.push(&encoded_buf[0..n]) {
                Ok(Some(report)) => defmt::todo!(),
                Ok(None) => defmt::todo!(),
                Err(_) => defmt::todo!(),
            }
        }
    }
    // read from host, decode, error handle
    // push stuff to a queue to be sent to the secondary pico if nescessary
    // control the actuators based on the command read from the host
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;
#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}
