use std::time::{Duration, Instant};

use bincode::{Decode, Encode};

extern crate cu_bincode as bincode;

#[derive(Encode, Decode, Debug)]
pub struct ClientMsg {
    pub seq: u64,
    pub payload: Vec<u8>,
}

#[derive(Encode, Decode, Debug)]
pub struct ServerMsg {
    pub seq: u64,
    pub payload: Vec<u8>,
}

const NUM_MESSAGES: u64 = 10_000;
const PAYLOAD_SIZE: usize = 1024;
const NUM_STREAMS: usize = 4;

fn handle_stream(handle: quic::StreamHandle<ServerMsg, ClientMsg>, stream_id: usize) -> u64 {
    let rt = tasker::get_tokio_handle();
    rt.block_on(async move {
        let reply_payload = vec![0xBBu8; PAYLOAD_SIZE];
        let mut received = 0u64;
        let start = Instant::now();

        loop {
            let msg = match handle.recv().await {
                Ok(msg) => msg,
                Err(e) => {
                    println!("[server][stream {stream_id}] recv ended: {e}");
                    break;
                }
            };

            received += 1;

            handle
                .send(&ServerMsg {
                    seq: msg.seq,
                    payload: reply_payload.clone(),
                })
                .await
                .expect("failed to send");

            if received % 1000 == 0 {
                println!("[server][stream {stream_id}] echoed {received} messages so far...");
            }

            if received >= NUM_MESSAGES {
                break;
            }
        }

        let elapsed = start.elapsed();
        println!(
            "[server][stream {stream_id}] done — echoed {received} in {:.2}s ({:.0} msg/s)",
            elapsed.as_secs_f64(),
            received as f64 / elapsed.as_secs_f64()
        );
        received
    })
}

fn main() {
    let rt = tasker::get_tokio_handle();
    rt.block_on(async {
        let server = quic::server::QuicServer::new(5000)
            .await
            .expect("failed to create server");

        println!("[server] listening on port 5000, expecting {NUM_STREAMS} streams");

        let conn = server
            .accept(Duration::from_millis(30), 0u8)
            .await
            .expect("failed to accept connection");
        println!("[server] client connected");

        let mut handles = Vec::new();
        for i in 0..NUM_STREAMS {
            let stream = conn
                .accept_bi::<ServerMsg, ClientMsg, 0>()
                .await
                .expect("failed to accept stream");
            handles.push((stream, i));
        }

        let threads: Vec<_> = handles
            .into_iter()
            .map(|(stream, id)| std::thread::spawn(move || handle_stream(stream, id)))
            .collect();

        let mut total = 0u64;
        for t in threads {
            total += t.join().unwrap();
        }

        println!("[server] all streams done — {total} total messages echoed");
        tasker::tokio::time::sleep(Duration::from_millis(500)).await;
    });
}
