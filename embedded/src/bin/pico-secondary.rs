#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::{
    adc::{Adc, Config as AdcConfig, InterruptHandler as AdcInterruptHandler, Channel},
    bind_interrupts,
    gpio::{Level, Output},
    peripherals::{UART1, UART0},
    uart::{Config as UartConfig, InterruptHandler as UartInterruptHandler, Uart},
};
use embedded_common::{SecondaryRequest, SecondaryResponse};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    UART0_IRQ => UartInterruptHandler<UART0>;
});

const UART_IDLE_TIMEOUT_MS: u64 = 1000;
const MUX_SETTLE_US: u64 = 10;


#[embassy_executor::main]
async fn main(spawner: Spawner) {
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
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(p.ADC, Irqs, adc_config);
    let mut adc_pin = Channel::new_pin(p.PIN_26, embassy_rp::gpio::Pull::None); // ADC0 is pin 26 
    
    
    // Initialize MUX  pins
    let mut mux_s0 = Output::new(p.PIN_19, Level::Low); // physical pin 25= GPIO 19
    let mut mux_s1 = Output::new(p.PIN_20, Level::Low); // physical pin 26= GPIO 20
    let mut mux_s2 = Output::new(p.PIN_21, Level::Low); // physical pin 27= GPIO 21
    let mut mux_s3 = Output::new(p.PIN_22, Level::Low); // physical pin 29 = GPIO22

    let mut rx_buf = [0u8; SecondaryRequest::SIZE];
    


    loop {
        // Wait for a request representing the MUX address
        match uart.read(&mut rx_buf).await {
            Ok(_) => {
                // Set MUX  pins
                mux = rx_buf[0];
                // Read ADC
                    match adc.read(&mut adc_pin).await {
                        Ok(value) => {
                            info!("ADC value: {}", value);
                            let resp = SecondaryResponse { adc_value: value };
                            let tx_buf = resp.serialize();
                            if let Err(e) = uart.write(&tx_buf).await {
                                error!("Failed to write UART response: {:?}", e);
                            }
                        }
                        Err(_e) => {
                            error!("ADC read failed");
                        }
                    }
                } else {
                    error!("Failed to deserialize request packet");
                }
            }
            Err(e) => {
                error!("UART read error: {:?}", e);
            }
        }
    }


fn set_mux(addr: u8, s0: &mut Output, s1: &mut Output, s2: &mut Output, s3: &mut Output) {
    s0.set_level(if addr & 0b0001 != 0 { Level::High } else { Level::Low });
    s1.set_level(if addr & 0b0010 != 0 { Level::High } else { Level::Low });
    s2.set_level(if addr & 0b0100 != 0 { Level::High } else { Level::Low });
    s3.set_level(if addr & 0b1000 != 0 { Level::High } else { Level::Low });
}
