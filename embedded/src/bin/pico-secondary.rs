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
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
    UART0_IRQ => UartInterruptHandler<UART0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    info!("Starting Secondary Pico");

    // Initialize UART for communication with main pico

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200;

    let mut uart = Uart::new(p.UART0, p.PIN_0, p.PIN_1, Irqs, p.DMA_CH0, p.DMA_CH1, uart_config);
    let mut rx_buf = [0u8; 1];
    let mut tx_buf = [0u8; 2];
    

    
    // Assuming UART0 on PIN_0 (tx) and PIN_1 (rx)


    // Initialize ADC
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(p.ADC, Irqs, adc_config);
    let mut adc_pin = Channel::new_pin(p.PIN_26, embassy_rp::gpio::Pull::None); // ADC0 is pin 26 
    
    
    // Initialize MUX  pins


    loop {
        // Wait for a request representing the MUX address
        match uart.read(&mut rx_buf).await 

                // Set MUX  pins
                // Read ADC
            Err(e) => {
                error!("UART read error: {:?}", e);
            }
        }
    }
}
