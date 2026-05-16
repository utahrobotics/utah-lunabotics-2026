use embassy_rp::uart::{Async, Uart};

pub async fn uart_write_cobs(uart: &mut Uart<'static, Async>, data: &[u8]) -> bool {
    let mut encoded = [0u8; 16];
    let len = cobs::encode(data, &mut encoded);
    encoded[len] = 0x00;
    uart.write(&encoded[..len + 1]).await.is_ok()
}

pub async fn uart_read_cobs(uart: &mut Uart<'static, Async>, out: &mut [u8]) -> bool {
    let mut encoded = [0u8; 16];
    let mut i = 0;
    loop {
        let mut b = [0u8; 1];
        if uart.read(&mut b).await.is_err() {
            return false;
        }
        if b[0] == 0x00 {
            break;
        }
        if i >= encoded.len() {
            return false; // frame too large, discard
        }
        encoded[i] = b[0];
        i += 1;
    }
    if i == 0 {
        return false;
    }
    cobs::decode(&encoded[..i], out).is_ok()
}
