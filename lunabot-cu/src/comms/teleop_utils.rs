use std::{
    sync::OnceLock,
    thread::JoinHandle,
    time::{Duration, Instant},
};

use common::{FromLunabase, FromLunabot};
use crossbeam::atomic::AtomicCell;
use crossbeam_channel::{Receiver, Sender};
use quic::QuicServer;

/// Timestamp of the last received message or keep-alive from Lunabase
static LAST_SEEN_TIMESTAMP: OnceLock<AtomicCell<Instant>> = OnceLock::new();

/// Provides a non blocking interface to the QuicServer struct
pub struct LunabaseConnection {
    pub server: QuicServer<FromLunabot, FromLunabase>,
    pub incoming_msg_queue: Receiver<FromLunabase>,
    pub outoging_msg_queue: Sender<FromLunabot>,
    pub recv_thread: JoinHandle<()>,
    pub send_thread: JoinHandle<()>,
    pub update_keep_alive_thread: JoinHandle<()>,
}

impl LunabaseConnection {
    pub fn new() -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let (tx_incoming, rx_incoming) = crossbeam_channel::unbounded();
        let (tx_outgoing, rx_outgoing) = crossbeam_channel::unbounded();

        let server = QuicServer::listen(common::ports::TELEOP, Duration::from_millis(100))?;

        LAST_SEEN_TIMESTAMP.get_or_init(|| AtomicCell::new(Instant::now()));

        // server is easily cloneable
        let server_c = server.clone();
        let recv_thread = std::thread::spawn(move || {
            loop {
                // recv messages in a loop,
                // send the messages to the rx
                // print a warning if there are more than 5 messages in the channel
                match server_c.recv() {
                    Ok(msg) => {
                        if tx_incoming.len() > 5 {
                            eprintln!(
                                "Warning: incoming message queue has {} messages",
                                tx_incoming.len()
                            );
                        }
                        if let Err(e) = tx_incoming.send(msg) {
                            eprintln!("Failed to send incoming message to queue: {}", e);
                            // break;
                        }
                    }
                    Err(_) => {
                        // eprintln!("Failed to receive message from server: {}", e);
                        std::thread::sleep(Duration::from_millis(10));
                    }
                }
            }
        });
        let server_c1 = server.clone();
        let update_keep_alive_thread = std::thread::spawn(move || {
            loop {
                std::thread::sleep(Duration::from_millis(20));
                if let Some(time_since) = server_c1.time_since_last_keep_alive() {
                    if let Some(timestamp) = LAST_SEEN_TIMESTAMP.get() {
                        let last_keep_alive = Instant::now() - time_since;
                        timestamp.store(last_keep_alive);
                    }
                }
            }
        });
        let server_c2 = server.clone();
        let send_thread = std::thread::spawn(move || {
            loop {
                // rx outgoing recv
                // send the message via the server_c1
                match rx_outgoing.recv() {
                    Ok(msg) => {
                        if let Err(_) = server_c2.send(msg) {
                            // eprintln!("Failed to send message via server: {}", e);
                        }
                    }
                    Err(e) => {
                        eprintln!("Outgoing message queue disconnected: {}", e);
                        break;
                    }
                }
            }
        });
        Ok(Self {
            server,
            incoming_msg_queue: rx_incoming,
            recv_thread,
            send_thread,
            outoging_msg_queue: tx_outgoing,
            update_keep_alive_thread,
        })
    }

    /// Try to receive a message from Lunabase without blocking
    pub fn try_recv(&self) -> Option<FromLunabase> {
        self.incoming_msg_queue.try_recv().ok()
    }

    /// Try to send a message to Lunabase without blocking
    pub fn try_send(
        &self,
        msg: FromLunabot,
    ) -> Result<(), crossbeam_channel::TrySendError<FromLunabot>> {
        self.outoging_msg_queue.try_send(msg)
    }

    /// Check if the connection is alive based on the last seen timestamp
    /// Returns true if a message or keep-alive was received within the given timeout
    pub fn is_alive(&self, timeout: Duration) -> bool {
        if let Some(timestamp) = LAST_SEEN_TIMESTAMP.get() {
            timestamp.load().elapsed() < timeout
        } else {
            false
        }
    }

    /// Get the time since last activity (message or keep-alive)
    pub fn time_since_last_seen() -> Option<Duration> {
        LAST_SEEN_TIMESTAMP.get().map(|ts| ts.load().elapsed())
    }
}
