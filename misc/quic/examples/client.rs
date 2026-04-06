use std::{net::SocketAddr, time::Instant};

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

fn run_stream(handle: quic::StreamHandle<ClientMsg, ServerMsg>, stream_id: usize) -> u64 {
    let rt = tasker::get_tokio_handle();
    rt.block_on(async move {
        let send_payload = vec![0xAAu8; PAYLOAD_SIZE];
        let start = Instant::now();
        let mut round_trips = 0u64;

        for seq in 0..NUM_MESSAGES {
            handle
                .send(&ClientMsg {
                    seq,
                    payload: send_payload.clone(),
                })
                .await
                .expect("failed to send");

            let resp = handle.recv().await.expect("failed to recv");
            assert_eq!(resp.seq, seq, "sequence mismatch on stream {stream_id}");
            round_trips += 1;

            if round_trips % 1000 == 0 {
                let elapsed = start.elapsed();
                println!(
                    "[client][stream {stream_id}] {round_trips} round-trips in {:.2}s ({:.0} rt/s)",
                    elapsed.as_secs_f64(),
                    round_trips as f64 / elapsed.as_secs_f64()
                );
            }
        }

        let elapsed = start.elapsed();
        println!(
            "[client][stream {stream_id}] done — {round_trips} round-trips in {:.2}s ({:.0} rt/s)",
            elapsed.as_secs_f64(),
            round_trips as f64 / elapsed.as_secs_f64()
        );
        round_trips
    })
}

fn main() {
    let rt = tasker::get_tokio_handle();
    rt.block_on(async {
        let bind_addr: SocketAddr = "0.0.0.0:0".parse().unwrap();
        let server_addr: SocketAddr = "127.0.0.1:5000".parse().unwrap();

        let client =
            quic::client::QuicClient::connect_insecure(bind_addr, server_addr, "localhost", 0u8)
                .await
                .expect("failed to connect");

        println!("[client] connected, opening {NUM_STREAMS} streams");

        let mut streams = Vec::new();
        for _ in 0..NUM_STREAMS {
            let stream = client
                .open_bi::<ClientMsg, ServerMsg, 0>()
                .await
                .expect("failed to open stream");
            streams.push(stream);
        }

        let start = Instant::now();

        let threads: Vec<_> = streams
            .into_iter()
            .enumerate()
            .map(|(id, stream)| std::thread::spawn(move || run_stream(stream, id)))
            .collect();

        let mut total = 0u64;
        for t in threads {
            total += t.join().unwrap();
        }

        let elapsed = start.elapsed();
        let total_bytes = total * 2 * (PAYLOAD_SIZE as u64 + 8);
        println!(
            "[client] all streams done — {total} total round-trips in {:.2}s ({:.0} rt/s, {:.1} MB/s)",
            elapsed.as_secs_f64(),
            total as f64 / elapsed.as_secs_f64(),
            total_bytes as f64 / elapsed.as_secs_f64() / 1_000_000.0
        );

        client.close();
    });
}
