#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler},
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::UART0,
    uart::{Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart},
};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_common::{SecondaryRequest, SecondaryResponse};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    UART0_IRQ => UartInterruptHandler<UART0>;
});

const UART_IDLE_TIMEOUT_MS: u64 = 1000;
const MUX_SETTLE_US: u64 = 10;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Starting Secondary Pico");

    // Initialize UART for communication with main pico

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200;

    let mut uart = Uart::new(
        p.UART0,
        p.PIN_12, // TX
        p.PIN_13, // RX
        Irqs,
        p.DMA_CH0,
        p.DMA_CH1,
        uart_config,
    );

    // Initialize ADC
    let adc_config = AdcConfig::default();
    let mut adc = Adc::new(p.ADC, Irqs, adc_config);
    let mut adc_pin = Channel::new_pin(p.PIN_27, embassy_rp::gpio::Pull::None); // ADC1 is pin 27

    // Initialize MUX  pins
    let mut mux_s0 = Output::new(p.PIN_19, Level::Low); // physical pin 25= GPIO 19
    let mut mux_s1 = Output::new(p.PIN_20, Level::Low); // physical pin 26= GPIO 20
    let mut mux_s2 = Output::new(p.PIN_21, Level::Low); // physical pin 27= GPIO 21
    let mut mux_s3 = Output::new(p.PIN_22, Level::Low); // physical pin 29 = GPIO22

    let mut rx_buf = [0u8; SecondaryRequest::SIZE];

    loop {
        match with_timeout(
            Duration::from_millis(UART_IDLE_TIMEOUT_MS),
            uart.read(&mut rx_buf),
        )
        .await
        {
            Err(_timeout) => {
                // Primary is idle or disconnected
                continue;
            }
            Ok(Err(e)) => {
                // UART error, next byte will correct
                error!("UART read error: {:?}", e);
                continue;
            }
            Ok(Ok(_)) => {}
        }

        let req = match SecondaryRequest::deserialize(rx_buf) {
            Ok(r) => r,
            Err(e) => {
                error!("Invalid request byte 0x{:02X}: {}", rx_buf[0], e);
                continue;
            }
        };

        info!("request mux channel {}", req.mux_address);

        set_mux(
            req.mux_address,
            &mut mux_s0,
            &mut mux_s1,
            &mut mux_s2,
            &mut mux_s3,
        );
        Timer::after(Duration::from_micros(MUX_SETTLE_US)).await;

        match adc.read(&mut adc_pin).await {
            Ok(value) => {
                info!("ADC ch{} = {}", req.mux_address, value);
                let tx_buf = SecondaryResponse { adc_value: value }.serialize();
                if let Err(e) = uart.write(&tx_buf).await {
                    error!("UART write error: {:?}", e);
                }
            }
            Err(_) => {
                error!("adc read failed for channel {}", req.mux_address);
            }
        }
    }
}

/// Bit 0 -> S0, bit 1 -> S1, bit 2 -> S2, bit 3 -> S3.
#[inline]
fn set_mux(
    addr: u8,
    s0: &mut Output<'_>,
    s1: &mut Output<'_>,
    s2: &mut Output<'_>,
    s3: &mut Output<'_>,
) {
    s0.set_level(if addr & 0b0001 != 0 {
        Level::High
    } else {
        Level::Low
    });
    s1.set_level(if addr & 0b0010 != 0 {
        Level::High
    } else {
        Level::Low
    });
    s2.set_level(if addr & 0b0100 != 0 {
        Level::High
    } else {
        Level::Low
    });
    s3.set_level(if addr & 0b1000 != 0 {
        Level::High
    } else {
        Level::Low
    });
}
