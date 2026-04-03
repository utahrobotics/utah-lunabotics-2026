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
use embassy_time::{Duration, Timer};
use embassy_usb::{
    UsbDevice,
    class::cdc_acm::{CdcAcmClass, Receiver, Sender, State},
};
use embedded_common::{ActuatorCommand, FromPico, MAX_MESSAGE_SIZE};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// something to read from usb
// something to forward to other pico
// something to read from sensors or other things

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

static PACKET_SIZE: u16 = 64;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let p = embassy_rp::init(Default::default());
    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);
    let config = {
        let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
        config.manufacturer = Some("USR");
        config.product = Some(embedded_common::PRIME_PICO_SERIAL);
        config.serial_number = Some(embedded_common::PRIME_PICO_SERIAL);
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

    // Build the builder.
    let usb = builder.build();

    info!("about to spawn usb task");
    // Run the USB device.
    spawner.spawn(usb_task(usb)).unwrap();

    let (class_tx, class_rx) = class.split();

    spawner.spawn(usb_tx_loop(class_tx)).unwrap();
    spawner.spawn(usb_rx_loop(class_rx)).unwrap();
    core::future::pending::<()>().await;
    defmt::unreachable!()
}

#[embassy_executor::task]
async fn usb_tx_loop(mut writer: Sender<'static, Driver<'static, USB>>) {
    // check if there is anything available from the secondary pico over UART (or maybe request information from the secondary pico?)
    // read (and decode?) the information off the secondary pico
    // send it over usb to the host
    loop {
        writer.wait_connection().await;
        'writer: while writer.dtr() {
            let mut stuffed = [0u8; cobs::max_encoding_length(FromPico::SIZE) + 1];
            let serialized = FromPico::Error.serialize();
            let len = cobs::encode(&serialized, &mut stuffed);
            for chunk in stuffed[..len + 1].chunks(16) {
                if let Err(e) = writer.write_packet(chunk).await {
                    error!("{:?}", e);
                    break 'writer;
                }
            }
            Timer::after(Duration::from_millis(100)).await;
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

#[embassy_executor::task]
async fn usb_rx_loop(mut reader: Receiver<'static, Driver<'static, USB>>) {
    let mut decoded_buf = [0u8; MAX_MESSAGE_SIZE];
    let mut encoded_buf = [0u8; PACKET_SIZE as usize];
    let mut decoder = cobs::CobsDecoder::new(&mut decoded_buf);
    loop {
        reader.wait_connection().await;
        while let Ok(n) = reader.read_packet(&mut encoded_buf).await {
            let mut remaining = &encoded_buf[..n];
            while !remaining.is_empty() {
                match decoder.push(remaining) {
                    Ok(Some(report)) => {
                        let frame = &decoder.dest()[..report.frame_size()];
                        if let Ok(frame_bytes) = <[u8; ActuatorCommand::SIZE]>::try_from(frame) {
                            match ActuatorCommand::deserialize(frame_bytes) {
                                Ok(cmd) => info!("received command: {}", cmd),
                                Err(e) => warn!("failed to deserialize ActuatorCommand: {}", e),
                            }
                        } else {
                            warn!(
                                "unexpected frame size: {} (expected {})",
                                report.frame_size(),
                                ActuatorCommand::SIZE
                            );
                        }
                        // advance past consumed bytes, reset decoder for next frame
                        remaining = &remaining[report.parsed_size()..];
                        decoder = cobs::CobsDecoder::new(&mut decoded_buf);
                    }
                    Ok(None) => break, // need more data
                    Err(_) => {
                        warn!("COBS decode error, resetting decoder");
                        decoder = cobs::CobsDecoder::new(&mut decoded_buf);
                        break;
                    }
                }
            }
        }
    }
}

type MyUsbDriver = Driver<'static, USB>;
type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;
#[embassy_executor::task]
async fn usb_task(mut usb: MyUsbDevice) -> ! {
    usb.run().await
}
