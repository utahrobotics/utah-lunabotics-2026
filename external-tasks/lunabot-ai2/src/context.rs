use std::thread::yield_now;

use bincode::{config::standard, decode_from_slice, encode_to_vec};
use common::{AI_HEARTBEAT_RATE, FromAI, FromHost};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::prelude::*;
use tokio::{
    sync::mpsc::{self, Receiver},
    time::{Duration, Instant},
};

use iceoryx_types::{FROM_AI_MAX_BYTES, FROM_HOST_MAX_BYTES, FromAIBytes, FromHostBytes};

const FROM_HOST_SERVICE: &str = "lunabot/host_to_ai";
const FROM_AI_SERVICE: &str = "lunabot/ai_to_host";

pub struct HostHandle {
    from_host: Receiver<FromHost>,
    publisher: Publisher<ipc::Service, FromAIBytes, ()>,
    last_heartbeat: Instant,
}

impl HostHandle {
    pub fn new() -> Self {
        let (from_host_tx, from_host) = mpsc::channel(32);

        std::thread::spawn(move || {
            let node = NodeBuilder::new()
                .create::<ipc::Service>()
                .expect("HostHandle: failed to create iceoryx2 node");

            let from_service = node
                .service_builder(
                    &ServiceName::new(FROM_HOST_SERVICE).expect("invalid service name"),
                )
                .publish_subscribe::<FromHostBytes>()
                .subscriber_max_buffer_size(20)
                .enable_safe_overflow(false)
                .open_or_create()
                .expect("HostHandle: failed to open host→AI service");

            let subscriber = from_service
                .subscriber_builder()
                .buffer_size(19)
                .create()
                .expect("HostHandle: failed to create subscriber");

            let config = standard();
            loop {
                match subscriber.receive() {
                    Ok(Some(sample)) => {
                        let payload: &FromHostBytes = &*sample;
                        let len = payload.len.min(FROM_HOST_MAX_BYTES as u32) as usize;
                        let bytes = &payload.data[..len];

                        match decode_from_slice::<FromHost, _>(bytes, config) {
                            Ok((msg, _)) => {
                                if let Err(e) = from_host_tx.try_send(msg) {
                                    let _ = from_host_tx.blocking_send(e.into_inner());
                                }
                            }
                            Err(e) => {
                                eprintln!("HostHandle: error decoding FromHostBytes: {}", e);
                                continue;
                            }
                        }
                    }
                    Ok(None) => {}
                    Err(e) => {
                        eprintln!("HostHandle: error receiving FromHostBytes: {}", e);
                    }
                }
            }
        });

        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .expect("HostHandle: failed to create iceoryx2 node (publisher)");

        let to_service = node
            .service_builder(&ServiceName::new(FROM_AI_SERVICE).expect("invalid service name"))
            .publish_subscribe::<FromAIBytes>()
            .subscriber_max_buffer_size(20)
            .enable_safe_overflow(false)
            .open_or_create()
            .expect("HostHandle: failed to open AI→host service");

        let publisher = to_service
            .publisher_builder()
            .unable_to_deliver_strategy(UnableToDeliverStrategy::Block)
            .create()
            .expect("HostHandle: failed to create publisher");

        Self {
            from_host,
            publisher,
            last_heartbeat: Instant::now(),
        }
    }

    pub async fn read_from_host(&mut self) -> FromHost {
        loop {
            let next_instant = self.last_heartbeat + AI_HEARTBEAT_RATE;
            tokio::select! {
                option = self.from_host.recv() => {
                    if let Some(msg) = option {
                        return msg;
                    }
                },
                _ = tokio::time::sleep_until(next_instant) => {
                    self.last_heartbeat = next_instant;
                    self.write_to_host(FromAI::Heartbeat);
                }
            }
        }
    }

    pub fn try_read_from_host(&mut self) -> Option<FromHost> {
        self.from_host.try_recv().ok()
    }

    // doesn't return an error because we dont care about the error anywhere else in the code
    pub fn write_to_host(&mut self, msg: FromAI) {
        let config = standard();
        if let Ok(bytes) = encode_to_vec(&msg, config) {
            if bytes.len() > FROM_AI_MAX_BYTES {
                eprintln!(
                    "HostHandle: message too large to send: {} bytes",
                    bytes.len()
                );
                return;
            }

            let mut payload = FromAIBytes::default();
            payload.len = bytes.len() as u32;
            payload.data[..bytes.len()].copy_from_slice(&bytes);
            match self.publisher.send_copy(payload) {
                Ok(_) => {}
                Err(e) => {
                    eprintln!("failed to send copy: {e}");
                }
            }
        }
    }
}
