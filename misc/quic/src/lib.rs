mod utils;
use std::{
    collections::VecDeque,
    marker::PhantomData,
    net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4},
    str::FromStr,
    sync::Arc,
};

use bincode::{Decode, Encode};
use cobs::CobsDecoder;
use quinn::{
    Connection, Endpoint, RecvStream, SendStream, crypto::rustls::QuicClientConfig, rustls,
};
use std::time::{Duration, Instant};
use tasker::{parking_lot::Mutex, tokio::io::AsyncWriteExt};

use crate::utils::{SkipServerVerification, make_server_endpoint};

/// Internal keep-alive packet format
#[derive(Debug, Clone, Encode, Decode)]
enum KeepAlivePacket {
    Ping(Vec<u8>),
    Pong(Vec<u8>),
}

pub struct InnerShared {
    pub connection: Option<Connection>,
    pub send_stream: Option<SendStream>,
    pub recv_stream: Option<RecvStream>,
    /// decoder has a box leaked buffer
    pub decoder: CobsDecoder<'static>,
    /// Queue of decoded messages waiting to be returned by recv()
    pub message_queue: VecDeque<Vec<u8>>,
    /// heartbeat message to send with ping/pong
    pub keep_alive_msg: Vec<u8>,
    pub keep_alive_interval_ms: u64,
    pub last_keep_alive_received: Option<Instant>,
    pub last_received_keep_alive_msg: Option<Vec<u8>>,
}

impl InnerShared {
    fn new() -> Self {
        let decoder_inner = Box::leak(Box::new(vec![0u8; 2 * 1024 * 1024]));
        let decoder = CobsDecoder::new(decoder_inner);
        Self {
            connection: None,
            send_stream: None,
            recv_stream: None,
            decoder,
            message_queue: VecDeque::new(),
            keep_alive_msg: Vec::new(),
            keep_alive_interval_ms: 150, // default 150ms
            last_keep_alive_received: None,
            last_received_keep_alive_msg: None,
        }
    }
}

/// Common function to receive and decode messages from a QUIC stream
async fn recv_messages(
    mut recv_stream: RecvStream,
    shared: Arc<Mutex<InnerShared>>,
    context: &str,
) -> Result<RecvStream, Box<dyn std::error::Error + Send + Sync>> {
    let mut buf = vec![0; 4096];
    let mut frames_decoded = 0;

    loop {
        match recv_stream.read(&mut buf).await {
            Ok(Some(byte_count)) => {
                if byte_count == 0 {
                    if frames_decoded > 0 {
                        return Ok(recv_stream);
                    }
                    return Err(Box::new(std::io::Error::other("Stream closed")));
                }
                let mut offset = 0;

                // Decode all frames in this read
                loop {
                    let mut shared_lock = shared.lock();

                    let decoded_frame_opt = match shared_lock.decoder.push(&buf[offset..byte_count])
                    {
                        Ok(Some(report)) => {
                            let frame_size = report.frame_size();
                            let parsed_size = report.parsed_size();

                            let frame_data = shared_lock.decoder.dest()[..frame_size].to_vec();
                            Some((frame_data, parsed_size))
                        }
                        Ok(None) => None,
                        Err(e) => {
                            eprintln!("[{context}] COBS decode error: {}", e);
                            return Err(Box::new(e));
                        }
                    };

                    if let Some((frame_data, parsed_size)) = decoded_frame_opt {
                        shared_lock.message_queue.push_back(frame_data);
                        frames_decoded += 1;
                        offset += parsed_size;
                        drop(shared_lock);
                        if offset >= byte_count {
                            break;
                        }
                    } else {
                        drop(shared_lock);
                        break;
                    }
                }

                // If we got at least one frame, we're done
                if frames_decoded > 0 {
                    return Ok(recv_stream);
                }
            }
            Ok(None) => {
                if frames_decoded > 0 {
                    return Ok(recv_stream);
                }
                return Err(Box::new(std::io::Error::other("Stream closed")));
            }
            Err(e) => {
                eprintln!("[{context}] Stream read error: {}", e);
                return Err(Box::new(std::io::Error::other(format!(
                    "Stream read error: {}",
                    e
                ))));
            }
        }
    }
}

/// Common function to handle bidirectional stream setup with handshake
async fn setup_bidirectional_stream(
    connection: Connection,
    shared: Arc<Mutex<InnerShared>>,
    is_server: bool,
    context: &str,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let (mut send_stream, mut recv_stream) = if is_server {
        connection.accept_bi().await?
    } else {
        let (send, recv) = connection.open_bi().await?;
        (send, recv)
    };

    // client writes first, server reads
    let mut handshake_buf = [0u8; 1];
    if is_server {
        recv_stream
            .read_exact(&mut handshake_buf)
            .await
            .map_err(|e| {
                eprintln!("[{context}] Failed to read initial handshake: {e}");
                e
            })?;
    } else {
        send_stream.write_all(&[0u8]).await.map_err(|e| {
            eprintln!("[{context}] Failed to write initial handshake: {e}");
            e
        })?;
        send_stream.flush().await.map_err(|e| {
            eprintln!("[{context}] Failed to flush initial handshake: {e}");
            e
        })?;
    }

    let mut shared_lock = shared.lock();
    shared_lock.send_stream = Some(send_stream);
    shared_lock.recv_stream = Some(recv_stream);
    shared_lock.connection = Some(connection);

    Ok(())
}

async fn send_keep_alive(
    connection: &Connection,
    packet: KeepAlivePacket,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let bytes = bincode::encode_to_vec(packet, bincode::config::standard())?;
    connection.send_datagram(bytes.into())?;
    Ok(())
}

async fn recv_keep_alive(
    connection: &Connection,
) -> Result<KeepAlivePacket, Box<dyn std::error::Error + Send + Sync>> {
    let datagram = connection.read_datagram().await?;
    let packet: KeepAlivePacket =
        bincode::decode_from_slice(&datagram, bincode::config::standard())?.0;
    Ok(packet)
}

fn start_server_keep_alive_task(connection: Connection, shared: Arc<Mutex<InnerShared>>) {
    tasker::get_tokio_handle().spawn(async move {
        loop {
            if connection.close_reason().is_some() {
                eprintln!("[SERVER] Connection closed, stopping keep-alive task");
                break;
            }
            match recv_keep_alive(&connection).await {
                Ok(KeepAlivePacket::Ping(msg)) => {
                    let pong_msg = {
                        let mut shared_lock = shared.lock();
                        shared_lock.last_keep_alive_received = Some(Instant::now());
                        shared_lock.last_received_keep_alive_msg = Some(msg);
                        shared_lock.keep_alive_msg.clone()
                    };

                    if let Err(e) =
                        send_keep_alive(&connection, KeepAlivePacket::Pong(pong_msg)).await
                    {
                        eprintln!("[SERVER] Failed to send pong: {e}");
                    }
                }
                Ok(KeepAlivePacket::Pong(_)) => {
                    eprintln!("[SERVER] Unexpected pong received, ignoring");
                }
                Err(e) => {
                    eprintln!("[SERVER] Failed to receive ping: {e}");
                    break;
                }
            }
        }
    });
}

fn start_client_keep_alive_task(
    connection: Connection,
    shared: Arc<Mutex<InnerShared>>,
    server_addr: SocketAddr,
    interval_ms: u64,
) {
    tasker::get_tokio_handle().spawn(async move {

        loop {
            if connection.close_reason().is_some() {
                eprintln!("[CLIENT] Connection closed, attempting reconnect...");

                if let Err(e) = attempt_client_reconnect(server_addr, Arc::clone(&shared)).await {
                    eprintln!("[CLIENT] Reconnection failed: {e}");
                    tasker::tokio::time::sleep(Duration::from_secs(5)).await;
                } else {
                    eprintln!("[CLIENT] Reconnected successfully");
                    if let Some(new_conn) = shared.lock().connection.clone() {
                        start_client_keep_alive_task(new_conn, Arc::clone(&shared), server_addr, interval_ms);
                        break;
                    }
                }
                continue;
            }

            tasker::tokio::time::sleep(Duration::from_millis(interval_ms)).await;

            let ping_msg = shared.lock().keep_alive_msg.clone();
            if let Err(e) = send_keep_alive(&connection, KeepAlivePacket::Ping(ping_msg)).await {
                eprintln!("[CLIENT] Failed to send ping: {e}");
                continue;
            }

            let timeout_duration = Duration::from_millis(interval_ms * 3);
            match tasker::tokio::time::timeout(timeout_duration, recv_keep_alive(&connection)).await
            {
                Ok(Ok(KeepAlivePacket::Pong(msg))) => {
                    let mut shared_lock = shared.lock();
                    shared_lock.last_keep_alive_received = Some(Instant::now());
                    shared_lock.last_received_keep_alive_msg = Some(msg);
                }
                Ok(Ok(KeepAlivePacket::Ping(_))) => {
                    eprintln!("[CLIENT] Unexpected ping received, ignoring");
                }
                Ok(Err(e)) => {
                    eprintln!("[CLIENT] Failed to receive pong: {e}");
                }
                Err(_) => {
                    eprintln!("[CLIENT] Pong timeout");
                }
            }
        }
    });
}

async fn attempt_client_reconnect(
    server_addr: SocketAddr,
    shared: Arc<Mutex<InnerShared>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut endpoint = Endpoint::client(SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0))?;

    let client_config = quinn::ClientConfig::new(Arc::new(QuicClientConfig::try_from(
        rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(SkipServerVerification::new())
            .with_no_client_auth(),
    )?));

    endpoint.set_default_client_config(client_config);

    let connecting = endpoint.connect(server_addr, "lunabot")?;
    let connection = connecting.await?;

    setup_bidirectional_stream(connection.clone(), Arc::clone(&shared), false, "CLIENT").await?;

    Ok(())
}

pub struct QuicServer<Msg: Encode + Decode<()>> {
    pub shared: Arc<Mutex<InnerShared>>,
    _boo: PhantomData<Msg>,
}

impl<Msg: Encode + Decode<()>> QuicServer<Msg> {
    /// Ensures the Tokio runtime is available for QUIC operations.
    fn ensure_runtime() {
        let _ = tasker::get_tokio_handle();
    }

    /// Listens on 0.0.0.0:{port}
    /// Accepts incoming connections
    pub fn listen(port: u32) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        Self::ensure_runtime();
        let (endpoint, _cert) = tasker::get_tokio_handle().block_on(async {
            make_server_endpoint(std::net::SocketAddr::V4(
                SocketAddrV4::from_str(&format!("0.0.0.0:{port}")).unwrap(),
            ))
        })?;

        let shared = Arc::new(Mutex::new(InnerShared::new()));
        let shared_c1 = Arc::clone(&shared);

        tasker::get_tokio_handle().spawn(async move {
            loop {
                let Some(incoming) = endpoint.accept().await else {
                    eprintln!("[SERVER] No Incoming Connection");
                    continue;
                };
                match incoming.accept() {
                    Ok(ongoing) => {
                        // 0 rrt should make re connections faster
                        let connection = ongoing.into_0rtt();
                        match connection {
                            Ok((connection, _zero_rtt_accepted)) => {
                                if let Err(e) = setup_bidirectional_stream(
                                    connection.clone(),
                                    Arc::clone(&shared_c1),
                                    true,
                                    "SERVER",
                                )
                                .await
                                {
                                    eprintln!("[SERVER] Failed to setup stream: {e}");
                                } else {
                                    start_server_keep_alive_task(
                                        connection,
                                        Arc::clone(&shared_c1),
                                    );
                                }
                            }
                            Err(ongoing) => {
                                eprintln!("[SERVER] 0-RTT Connection failed, attempting to establish normal connection");
                                match ongoing.await {
                                    Ok(connection) => {
                                        if let Err(e) = setup_bidirectional_stream(
                                            connection.clone(),
                                            Arc::clone(&shared_c1),
                                            true,
                                            "SERVER",
                                        )
                                        .await
                                        {
                                            eprintln!("[SERVER] Failed to setup stream: {e}");
                                        } else {
                                            start_server_keep_alive_task(
                                                connection,
                                                Arc::clone(&shared_c1),
                                            );
                                        }
                                    }
                                    Err(e) => {
                                        eprintln!("[SERVER] Failed to establish connection: {e}");
                                    }
                                }
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("[SERVER] Failed to accept incoming connection: {e}");
                    }
                }
            }
        });

        Ok(Self {
            shared,
            _boo: PhantomData {},
        })
    }

    /// blocking operation
    pub fn recv(&self) -> Result<Msg, Box<dyn std::error::Error + Send + Sync>> {
        recv_common(&self.shared, "SERVER")
    }

    /// blocking operation
    pub fn send(&self, packet: Msg) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        send_common(&self.shared, packet)
    }

    pub fn is_connected(&self) -> bool {
        let connection = &self.shared.lock().connection;
        connection.is_some() && connection.as_ref().unwrap().close_reason().is_none()
    }

    pub fn get_last_keep_alive_msg(&self) -> Option<Vec<u8>> {
        self.shared.lock().last_received_keep_alive_msg.clone()
    }

    pub fn time_since_last_keep_alive(&self) -> Option<Duration> {
        self.shared
            .lock()
            .last_keep_alive_received
            .map(|t| t.elapsed())
    }
}

pub struct QuicClient<Msg: Encode + Decode<()> + Clone> {
    pub shared: Arc<Mutex<InnerShared>>,
    _boo: PhantomData<Msg>,
}

impl<Msg: Encode + Decode<()> + Clone> QuicClient<Msg> {
    fn ensure_runtime() {
        let _ = tasker::get_tokio_handle();
    }

    /// Connect to a server
    /// the keep alive packet is typically used to communicate the lunabot stage.
    pub fn connect(
        server_addr: SocketAddr,
        initial_keep_alive_packet: &[u8],
        keep_alive_interval: Duration,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        Self::ensure_runtime();

        let shared = Arc::new(Mutex::new(InnerShared::new()));
        {
            let mut shared_lock = shared.lock();
            shared_lock.keep_alive_msg = initial_keep_alive_packet.to_vec();
            shared_lock.keep_alive_interval_ms = keep_alive_interval.as_millis() as u64;
        }

        let shared_c1 = Arc::clone(&shared);

        tasker::get_tokio_handle().spawn(async move {
            let mut endpoint =
                match Endpoint::client(SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 0)) {
                    Ok(ep) => ep,
                    Err(e) => {
                        eprintln!("[CLIENT] Failed to create client endpoint: {e}");
                        return;
                    }
                };

            let client_config = match quinn::ClientConfig::new(Arc::new(
                match QuicClientConfig::try_from(
                    rustls::ClientConfig::builder()
                        .dangerous()
                        .with_custom_certificate_verifier(SkipServerVerification::new())
                        .with_no_client_auth(),
                ) {
                    Ok(config) => config,
                    Err(e) => {
                        eprintln!("[CLIENT] Failed to create client config: {e}");
                        return;
                    }
                },
            )) {
                config => config,
            };

            endpoint.set_default_client_config(client_config);

            match endpoint.connect(server_addr, "lunabot") {
                Ok(connecting) => match connecting.await {
                    Ok(connection) => {
                        if let Err(e) = setup_bidirectional_stream(
                            connection.clone(),
                            Arc::clone(&shared_c1),
                            false,
                            "CLIENT",
                        )
                        .await
                        {
                            eprintln!("[CLIENT] Failed to setup stream: {e}");
                        } else {
                            start_client_keep_alive_task(
                                connection,
                                Arc::clone(&shared_c1),
                                server_addr,
                                keep_alive_interval.as_millis() as u64,
                            );
                        }
                    }
                    Err(e) => {
                        eprintln!("[CLIENT] Failed to establish connection: {e}");
                    }
                },
                Err(e) => {
                    eprintln!("[CLIENT] Failed to connect to server: {e}");
                }
            }
        });

        Ok(Self {
            shared,
            _boo: PhantomData {},
        })
    }

    /// blocking operation
    pub fn recv(&self) -> Result<Msg, Box<dyn std::error::Error + Send + Sync>> {
        recv_common(&self.shared, "CLIENT")
    }

    /// blocking operation
    pub fn send(&self, packet: Msg) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        send_common(&self.shared, packet)
    }

    pub fn is_connected(&self) -> bool {
        let connection = &self.shared.lock().connection;
        connection.is_some() && connection.as_ref().unwrap().close_reason().is_none()
    }

    /// typically used to communicate the lunabot stage
    pub fn set_keep_alive_msg(&self, msg: &[u8]) {
        self.shared.lock().keep_alive_msg = msg.to_vec();
    }

    pub fn get_last_keep_alive_msg(&self) -> Option<Vec<u8>> {
        self.shared.lock().last_received_keep_alive_msg.clone()
    }

    pub fn time_since_last_keep_alive(&self) -> Option<Duration> {
        self.shared
            .lock()
            .last_keep_alive_received
            .map(|t| t.elapsed())
    }
}

/// Common recv used by client and server
fn recv_common<Msg: Encode + Decode<()>>(
    shared: &Arc<Mutex<InnerShared>>,
    // context only used for logs, otherwise I would have used an enum
    context: &str,
) -> Result<Msg, Box<dyn std::error::Error + Send + Sync>> {
    // Return a message from the queue if one is already there
    {
        let mut shared_lock = shared.lock();
        if let Some(msg_bytes) = shared_lock.message_queue.pop_front() {
            return Ok(bincode::decode_from_slice(&msg_bytes, bincode::config::standard())?.0);
        }
    }

    // read from stream
    let recv_stream_opt = shared.lock().recv_stream.take();
    if let Some(recv_stream) = recv_stream_opt {
        let shared_c = Arc::clone(shared);
        let result = tasker::get_tokio_handle()
            .block_on(async move { recv_messages(recv_stream, shared_c, context).await });

        match result {
            Ok(stream) => {
                shared.lock().recv_stream = Some(stream);
            }
            Err(e) => {
                return Err(e);
            }
        }

        let mut shared_lock = shared.lock();
        if let Some(msg_bytes) = shared_lock.message_queue.pop_front() {
            Ok(bincode::decode_from_slice(&msg_bytes, bincode::config::standard())?.0)
        } else {
            Err(Box::new(std::io::Error::other("No message received")))
        }
    } else {
        Err(Box::new(std::io::Error::other(
            "No established connection.",
        )))
    }
}

/// Common send used by client and server
fn send_common<Msg: Encode + Decode<()>>(
    shared: &Arc<Mutex<InnerShared>>,
    packet: Msg,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let bytes = bincode::encode_to_vec(packet, bincode::config::standard())?;
    if let Some(ref mut send_stream) = shared.lock().send_stream {
        tasker::get_tokio_handle().block_on(async {
            send_stream
                .write_all(&cobs::encode_vec_including_sentinels(&bytes))
                .await?;
            send_stream.flush().await
        })?;
    } else {
        return Err(Box::new(std::io::Error::other(
            "No established connection.",
        )));
    }
    Ok(())
}
