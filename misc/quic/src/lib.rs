use std::{
    collections::VecDeque,
    marker::PhantomData,
    sync::{Arc, Mutex},
    time::{Duration, Instant},
};

use bincode::{Decode, Encode};
use cobs::CobsDecoderOwned;
use quinn::{Connection, ReadError, RecvStream, SendStream, WriteError};
use tasker::tokio;

extern crate cu_bincode as bincode;

pub mod client;
pub mod server;
pub mod utils;

// ── Keep-alive types ──────────────────────────────────────────────────

/// Wire format for the datagram ping/pong.
#[derive(Debug, Clone, Encode, Decode)]
pub enum KeepAlivePacket<KA: Encode + Decode<()>> {
    Ping(KA),
    Pong(KA),
}

/// Shared mutable state for the keep-alive mechanism.
pub struct KeepAliveState<KA> {
    /// The message we attach to outgoing pings/pongs.
    pub outgoing_msg: KA,
    /// The payload from the last ping/pong we received.
    pub last_received_msg: Option<KA>,
    /// When we last received a ping or pong.
    pub last_received_at: Option<Instant>,
}

impl<KA: Clone> KeepAliveState<KA> {
    pub fn last_msg(&self) -> Option<KA> {
        self.last_received_msg.clone()
    }

    pub fn time_since_last(&self) -> Option<Duration> {
        self.last_received_at.map(|t| t.elapsed())
    }
}

/// Handle returned after starting keep-alive on a connection.
///
/// Dropping the handle aborts the background task.
pub struct KeepAlive<KA> {
    state: Arc<Mutex<KeepAliveState<KA>>>,
    abort_handle: tokio::task::AbortHandle,
}

impl<KA> Drop for KeepAlive<KA> {
    fn drop(&mut self) {
        self.abort_handle.abort();
    }
}

impl<KA: Clone> KeepAlive<KA> {
    /// Set the message that will be sent with future pings/pongs.
    pub fn set_msg(&self, msg: KA) {
        self.state.lock().unwrap().outgoing_msg = msg;
    }

    /// Get the last received keep-alive payload (from the remote's ping or pong).
    pub fn last_msg(&self) -> Option<KA> {
        self.state.lock().unwrap().last_received_msg.clone()
    }

    /// How long since we last heard from the remote.
    pub fn time_since_last(&self) -> Option<Duration> {
        self.state.lock().unwrap().last_received_at.map(|t| t.elapsed())
    }

    /// Get a shared reference to the inner state, allowing reads without owning
    /// the `KeepAlive` handle.
    pub fn shared_state(&self) -> Arc<Mutex<KeepAliveState<KA>>> {
        Arc::clone(&self.state)
    }
}

fn send_datagram_packet<KA: Encode + Decode<()>>(
    connection: &Connection,
    packet: KeepAlivePacket<KA>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let bytes = bincode::encode_to_vec(packet, bincode::config::standard())?;
    connection.send_datagram(bytes.into())?;
    Ok(())
}

fn decode_datagram_packet<KA: Encode + Decode<()>>(
    data: &[u8],
) -> Result<KeepAlivePacket<KA>, Box<dyn std::error::Error + Send + Sync>> {
    Ok(bincode::decode_from_slice(data, bincode::config::standard())?.0)
}

/// a few dropped datagrams are not necessarily fatal
const MAX_CONSECUTIVE_PONG_MISSES: u32 = 5;

/// Start the **server-side** keep-alive loop (sends pings, expects pongs).
pub fn start_server_keep_alive<KA>(
    connection: Connection,
    initial_msg: KA,
    interval: Duration,
) -> KeepAlive<KA>
where
    KA: Encode + Decode<()> + Clone + Send + Sync + 'static,
{
    let state = Arc::new(Mutex::new(KeepAliveState {
        outgoing_msg: initial_msg,
        last_received_msg: None,
        last_received_at: None,
    }));
    let state_c = Arc::clone(&state);

    let handle = tokio::spawn(async move {
        let mut ticker = tokio::time::interval(interval);
        // First tick completes immediately; skip it so we don't ping at t=0.
        ticker.tick().await;

        let mut consecutive_misses: u32 = 0;

        loop {
            ticker.tick().await;

            if connection.close_reason().is_some() {
                break;
            }

            let ping_msg = state_c.lock().unwrap().outgoing_msg.clone();
            if let Err(e) = send_datagram_packet(&connection, KeepAlivePacket::Ping(ping_msg)) {
                eprintln!("[SERVER keep-alive] failed to send ping: {e}");
                continue;
            }

            // Wait up to 3x interval for a pong
            let timeout = interval * 3;
            match tokio::time::timeout(timeout, connection.read_datagram()).await {
                Ok(Ok(data)) => {
                    consecutive_misses = 0;
                    match decode_datagram_packet::<KA>(&data) {
                        Ok(KeepAlivePacket::Pong(msg)) => {
                            let mut s = state_c.lock().unwrap();
                            s.last_received_msg = Some(msg);
                            s.last_received_at = Some(Instant::now());
                        }
                        Ok(KeepAlivePacket::Ping(_)) => {
                            eprintln!("[SERVER keep-alive] unexpected ping, ignoring");
                        }
                        Err(e) => {
                            eprintln!("[SERVER keep-alive] decode error: {e}");
                        }
                    }
                }
                Ok(Err(e)) => {

                    eprintln!(
                        "[SERVER keep-alive] datagram read error: {e}, closing connection"
                    );
                    connection.close(1u32.into(), b"keep-alive read error");
                    break;
                }
                Err(_) => {
                    consecutive_misses += 1;
                    eprintln!(
                        "[SERVER keep-alive] pong timeout ({}/{})",
                        consecutive_misses, MAX_CONSECUTIVE_PONG_MISSES
                    );
                    if consecutive_misses >= MAX_CONSECUTIVE_PONG_MISSES {
                        eprintln!(
                            "[SERVER keep-alive] {MAX_CONSECUTIVE_PONG_MISSES} consecutive pong misses, closing connection"
                        );
                        connection.close(1u32.into(), b"pong timeout");
                        break;
                    }
                }
            }
        }
    });

    KeepAlive {
        state,
        abort_handle: handle.abort_handle(),
    }
}

/// Start the **client-side** keep-alive loop (listens for pings, replies with pongs).
pub fn start_client_keep_alive<KA>(
    connection: Connection,
    initial_msg: KA,
) -> KeepAlive<KA>
where
    KA: Encode + Decode<()> + Clone + Send + Sync + 'static,
{
    let state = Arc::new(Mutex::new(KeepAliveState {
        outgoing_msg: initial_msg,
        last_received_msg: None,
        last_received_at: None,
    }));
    let state_c = Arc::clone(&state);

    let handle = tokio::spawn(async move {
        loop {
            if connection.close_reason().is_some() {
                break;
            }

            match connection.read_datagram().await {
                Ok(data) => match decode_datagram_packet::<KA>(&data) {
                    Ok(KeepAlivePacket::Ping(msg)) => {
                        let pong_msg;
                        {
                            let mut s = state_c.lock().unwrap();
                            s.last_received_msg = Some(msg);
                            s.last_received_at = Some(Instant::now());
                            pong_msg = s.outgoing_msg.clone();
                        }
                        if let Err(e) = send_datagram_packet(
                            &connection,
                            KeepAlivePacket::Pong(pong_msg),
                        ) {
                            eprintln!("[CLIENT keep-alive] failed to send pong: {e}");
                        }
                    }
                    Ok(KeepAlivePacket::Pong(_)) => {
                        eprintln!("[CLIENT keep-alive] unexpected pong, ignoring");
                    }
                    Err(e) => {
                        eprintln!("[CLIENT keep-alive] decode error: {e}");
                    }
                },
                Err(e) => {
                    eprintln!("[CLIENT keep-alive] datagram read error: {e}");
                    break;
                }
            }
        }
    });

    KeepAlive {
        state,
        abort_handle: handle.abort_handle(),
    }
}

#[derive(Clone)]
pub struct StreamHandle<Outgoing: Encode + Decode<()>, Incoming: Encode + Decode<()>> {
    shared_recv: Arc<tokio::sync::Mutex<InnerSharedReader>>,
    shared_send: Arc<tokio::sync::Mutex<InnerSharedWriter>>,
    _boo: PhantomData<Outgoing>,
    _boo_incoming: PhantomData<Incoming>,
}

struct InnerSharedReader {
    recv_stream: RecvStream,
    decoder: CobsDecoderOwned,
    message_queue: VecDeque<Vec<u8>>,
}

struct InnerSharedWriter {
    send_stream: SendStream,
}

const DECODER_BUF_SIZE: usize = 4096;

impl<Outgoing: Encode + Decode<()>, Incoming: Encode + Decode<()>>
    StreamHandle<Outgoing, Incoming>
{
    pub(crate) fn new(send: SendStream, recv: RecvStream) -> Self {
        Self {
            shared_recv: Arc::new(tokio::sync::Mutex::new(InnerSharedReader {
                recv_stream: recv,
                decoder: CobsDecoderOwned::new(DECODER_BUF_SIZE),
                message_queue: VecDeque::new(),
            })),
            shared_send: Arc::new(tokio::sync::Mutex::new(InnerSharedWriter {
                send_stream: send,
            })),
            _boo: PhantomData,
            _boo_incoming: PhantomData,
        }
    }

    /// Reads from the QUIC recv stream, feeds bytes through the COBS decoder,
    /// and returns the next decoded + deserialized message.
    pub async fn recv(&self) -> Result<Incoming, ReadError> {
        loop {
            let mut reader = self.shared_recv.lock().await;
            if let Some(msg_bytes) = reader.message_queue.pop_front() {
                let (msg, _) = bincode::decode_from_slice::<Incoming, _>(
                    &msg_bytes,
                    bincode::config::standard(),
                )
                .map_err(|_| ReadError::Reset(0u32.into()))?;
                return Ok(msg);
            }
            let mut buf = [0u8; 4096];
            let read_result = reader.recv_stream.read(&mut buf).await;
            match read_result? {
                None => {
                    return Err(ReadError::Reset(0u32.into()));
                }
                Some(n) => {
                    let mut offset = 0;
                    while offset < n {
                        match reader.decoder.push(&buf[offset..n]) {
                            Ok(Some(report)) => {
                                let frame = reader.decoder.dest()[..report.frame_size()].to_vec();
                                reader.message_queue.push_back(frame);
                                offset += report.parsed_size();
                                reader.decoder.reset();
                            }
                            Ok(None) => {
                                break;
                            }
                            Err(_) => {
                                reader.decoder.reset();
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    /// Serializes the message with bincode, COBS-encodes it with a trailing
    /// zero delimiter, and writes it to the QUIC send stream.
    pub async fn send(&self, msg: &Outgoing) -> Result<(), WriteError> {
        let bincode_bytes = bincode::encode_to_vec(msg, bincode::config::standard())
            .expect("bincode encode failed");

        let mut cobs_frame = cobs::encode_vec(&bincode_bytes);
        cobs_frame.push(0x00); // frame delimiter

        let mut writer = self.shared_send.lock().await;
        writer.send_stream.write_all(&cobs_frame).await
    }
}
