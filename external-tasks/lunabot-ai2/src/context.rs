use bincode::{config::standard, decode_from_slice, encode_to_vec};
use common::{FromAI, FromHost, AI_HEARTBEAT_RATE};
use iceoryx2::node::NodeBuilder;
use iceoryx2::port::publisher::Publisher;
use iceoryx2::prelude::*;
use tokio::{sync::mpsc::{self, Receiver}, time::{Instant, Duration}};

use iceoryx_types::{FromAIBytes, FromHostBytes, FROM_AI_MAX_BYTES, FROM_HOST_MAX_BYTES};

const FROM_HOST_SERVICE: &str = "lunabot/host_to_ai";
const FROM_AI_SERVICE: &str = "lunabot/ai_to_host";

pub struct HostHandle {
    from_host: Receiver<FromHost>,
    publisher: Publisher<ipc::Service, FromAIBytes, ()>,
    last_heartbeat: Instant,
}

impl HostHandle {
    pub fn new() -> Self {
        // Channel used to forward decoded messages into async context
        let (from_host_tx, from_host) = mpsc::channel(32);

        // Build iceoryx2 node and ports in a dedicated thread (blocking API)
        std::thread::spawn(move || {
            // Create node
            let node = NodeBuilder::new()
                .create::<ipc::Service>()
                .expect("HostHandle: failed to create iceoryx2 node");

            // --- Subscriber for FromHostBytes ---
            let from_service = node
                .service_builder(&ServiceName::new(FROM_HOST_SERVICE).expect("invalid service name"))
                .publish_subscribe::<FromHostBytes>()
                .open_or_create()
                .expect("HostHandle: failed to open host→AI service");

            let subscriber = from_service
                .subscriber_builder()
                .create()
                .expect("HostHandle: failed to create subscriber");

            // Blocking loop: receive samples, decode, forward through channel
            let config = standard();
            loop {
                match subscriber.receive() {
                    Ok(Some(sample)) => {
                        let payload: &FromHostBytes = &*sample;
                        let len = payload.len.min(FROM_HOST_MAX_BYTES as u32) as usize;
                        let bytes = &payload.data[..len];

                        match decode_from_slice::<FromHost, _>(bytes, config) {
                            Ok((msg, _)) => {
                                // Non-blocking send; fall back to blocking if full.
                                if let Err(e) = from_host_tx.try_send(msg) {
                                    let _ = from_host_tx.blocking_send(e.into_inner());
                                }
                            }
                            Err(_) => {
                                // Could log decode error, skip invalid message
                                continue;
                            }
                        }
                    }
                    Ok(None) => {
                        // No new sample; yield a bit
                        std::thread::sleep(Duration::from_millis(5));
                    }
                    Err(_e) => {
                        // Error retrieving data – in production we might log; here just continue
                        std::thread::sleep(Duration::from_millis(5));
                    }
                }
            }
        });

        // Create publisher inside the async context (we can't share Node across threads easily)
        let node = NodeBuilder::new()
            .create::<ipc::Service>()
            .expect("HostHandle: failed to create iceoryx2 node (publisher)");

        let to_service = node
            .service_builder(&ServiceName::new(FROM_AI_SERVICE).expect("invalid service name"))
            .publish_subscribe::<FromAIBytes>()
            .open_or_create()
            .expect("HostHandle: failed to open AI→host service");

        let publisher = to_service
            .publisher_builder()
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
                    // Send heartbeat and update timer
                    self.last_heartbeat = next_instant;
                    self.write_to_host(FromAI::Heartbeat);
                }
            }
        }
    }

    pub fn try_read_from_host(&mut self) -> Option<FromHost> {
        self.from_host.try_recv().ok()
    }

    pub fn write_to_host(&mut self, msg: FromAI) {
        let config = standard();
        if let Ok(bytes) = encode_to_vec(&msg, config) {
            if bytes.len() > FROM_AI_MAX_BYTES {
                // Infeasible large message; drop.
                return;
            }

            let mut payload = FromAIBytes::default();
            payload.len = bytes.len() as u32;
            payload.data[..bytes.len()].copy_from_slice(&bytes);

            if let Ok(sample) = self.publisher.loan_uninit() {
                let initialized = sample.write_payload(payload);
                let _ = initialized.send();
            }
        }
    }
}
