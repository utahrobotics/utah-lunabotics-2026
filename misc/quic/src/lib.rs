mod utils;
use std::{
    collections::VecDeque,
    marker::PhantomData,
    net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4},
    str::FromStr,
    sync::Arc,
};

use bincode::{Decode, Encode};
use cobs::{CobsDecoderOwned};
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
    pub decoder: CobsDecoderOwned,
    /// Queue of decoded messages waiting to be returned by recv()
    pub message_queue: VecDeque<Vec<u8>>,
    /// heartbeat message to send with ping/pong
    pub keep_alive_msg: Vec<u8>,
    pub last_keep_alive_received: Option<Instant>,
    pub last_received_keep_alive_msg: Option<Vec<u8>>,
    /// Abort handle to kill the current keep-alive task when new connection accepted
    pub keep_alive_abort_handle: Option<tasker::tokio::task::AbortHandle>,
    /// Watch channel sender to signal new connection (cancels pending recvs)
    pub new_connection_tx: tasker::tokio::sync::watch::Sender<u64>,
    /// Watch channel receiver to detect new connections
    /// when a new connection is detected, the decoder is reset, and the message queue is cleared, and any recv operations are cancelled
    pub new_connection_rx: tasker::tokio::sync::watch::Receiver<u64>,
}

impl InnerShared {
    fn new() -> Self {
        // 2MB decoder buffer
        let decoder = CobsDecoderOwned::new(1024*1024*2);
        let (new_connection_tx, new_connection_rx) = tasker::tokio::sync::watch::channel(0u64);
        Self {
            connection: None,
            send_stream: None,
            recv_stream: None,
            decoder,
            message_queue: VecDeque::new(),
            keep_alive_msg: Vec::new(),
            last_keep_alive_received: None,
            last_received_keep_alive_msg: None,
            keep_alive_abort_handle: None,
            new_connection_tx,
            new_connection_rx,
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

    // this loop is broken out of once a whole frame is seen
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
    
    let is_replacement = shared_lock.connection.is_some();
    
    shared_lock.message_queue.clear();
    
    
    shared_lock.send_stream = Some(send_stream);
    shared_lock.recv_stream = Some(recv_stream);
    shared_lock.connection = Some(connection);
    
    // signal if we're replacing an existing connection
    // This wakes up any blocked recv() calls to use the new connection
    if is_replacement {
        let current_gen = *shared_lock.new_connection_tx.borrow();
        let new_gen = current_gen.wrapping_add(1);
        shared_lock.new_connection_tx.send(new_gen).ok();
        shared_lock.decoder.reset();
    }

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

fn start_server_keep_alive_task(
    connection: Connection,
    shared: Arc<Mutex<InnerShared>>,
    interval_ms: u64,
) -> tasker::tokio::task::AbortHandle {
    let handle = tasker::get_tokio_handle().spawn(async move {
        loop {
            if connection.close_reason().is_some() {
                eprintln!("[SERVER] Connection closed, stopping keep-alive task");
                break;
            }

            tasker::tokio::time::sleep(Duration::from_millis(interval_ms)).await;


            let ping_msg = shared.lock().keep_alive_msg.clone();
            if let Err(e) = send_keep_alive(&connection, KeepAlivePacket::Ping(ping_msg)).await {
                eprintln!("[SERVER] Failed to send ping ({})", e);
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
                    eprintln!("[SERVER] Unexpected ping received, ignoring");
                }
                Ok(Err(e)) => {
                    eprintln!("[SERVER] Failed to receive pong: {e}");
                }
                Err(_) => {
                    eprintln!("[SERVER] Pong timeout");
                }
            }
        }
    });
    
    handle.abort_handle()
}

fn start_client_keep_alive_task(
    connection: Connection,
    shared: Arc<Mutex<InnerShared>>,
) {
    tasker::get_tokio_handle().spawn(async move {
        loop {
            if connection.close_reason().is_some() {
                eprintln!("[CLIENT] Connection closed, stopping keep-alive task");
                break;
            }

            match recv_keep_alive(&connection).await {
                Ok(KeepAlivePacket::Ping(msg)) => {
                    {
                        let mut shared_lock = shared.lock();
                        shared_lock.last_keep_alive_received = Some(Instant::now());
                        shared_lock.last_received_keep_alive_msg = Some(msg.clone());
                    }

                    if let Err(e) = send_keep_alive(&connection, KeepAlivePacket::Pong(msg)).await {
                        eprintln!("[CLIENT] Failed to send pong ({})", e);
                    }
                }
                Ok(KeepAlivePacket::Pong(_)) => {
                    eprintln!("[CLIENT] Unexpected pong received, ignoring");
                }
                Err(e) => {
                    eprintln!("[CLIENT] Failed to receive ping ({})", e);
                }
            }
        }
    });
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
    pub fn listen(port: u32, keep_alive_interval: Duration) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        Self::ensure_runtime();
        let (endpoint, _cert) = tasker::get_tokio_handle().block_on(async {
            make_server_endpoint(std::net::SocketAddr::V4(
                SocketAddrV4::from_str(&format!("0.0.0.0:{port}")).unwrap(),
            ))
        })?;

        let shared = Arc::new(Mutex::new(InnerShared::new()));
        let shared_c1 = Arc::clone(&shared);
        let interval_ms = keep_alive_interval.as_millis() as u64;
        println!("Listening on {}",format!("0.0.0.0:{port}"));
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
                                // abort old keep-alive task if it exists
                                if let Some(old_handle) = shared_c1.lock().keep_alive_abort_handle.take() {
                                    old_handle.abort();
                                    eprintln!("[SERVER] Aborted old keep-alive task (new connection accepted)");
                                }
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
                                    // start new keep-alive task, store handle
                                    let abort_handle = start_server_keep_alive_task(
                                        connection,
                                        Arc::clone(&shared_c1),
                                        interval_ms,
                                    );
                                    shared_c1.lock().keep_alive_abort_handle = Some(abort_handle);
                                }
                            }
                            Err(ongoing) => {
                                eprintln!("[SERVER] 0-RTT Connection failed, attempting to establish normal connection");
                                match ongoing.await {
                                    Ok(connection) => {
                                        // abort old keep-alive task if it exists
                                        if let Some(old_handle) = shared_c1.lock().keep_alive_abort_handle.take() {
                                            old_handle.abort();
                                            eprintln!("[SERVER] Aborted old keep-alive task (new connection accepted)");
                                        }
                                        
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
                                            // start new keep-alive task, store handle
                                            let abort_handle = start_server_keep_alive_task(
                                                connection,
                                                Arc::clone(&shared_c1),
                                                interval_ms,
                                            );
                                            shared_c1.lock().keep_alive_abort_handle = Some(abort_handle);
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
    /// only send messages of under 1 mb otherwise this will likely fail
    pub fn send(&self, packet: Msg) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        send_common(&self.shared, packet)
    }

    /// checks if the connection has not been reported as closed
    /// you should not use this to check if the connection is alive, use the last seen keep-alive mechanism instead
    /// this is only useful for detecting if the connection has been closed by the remote end
    pub fn is_connected(&self) -> bool {
        let connection = &self.shared.lock().connection;
        connection.is_some() && connection.as_ref().unwrap().close_reason().is_none()
    }

    pub fn get_last_keep_alive_msg(&self) -> Option<Vec<u8>> {
        self.shared.lock().last_received_keep_alive_msg.clone()
    }

    /// typically used to communicate the lunabot stage
    pub fn set_keep_alive_msg(&self, msg: &[u8]) {
        self.shared.lock().keep_alive_msg = msg.to_vec();
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
    /// does not block if the server isn't available
    pub fn connect(
        server_addr: SocketAddr,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        Self::ensure_runtime();

        let shared = Arc::new(Mutex::new(InnerShared::new()));
        {
            let mut shared_lock = shared.lock();
            shared_lock.keep_alive_msg = [0u8].to_vec();
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

            let mut client_config = match quinn::ClientConfig::new(Arc::new(
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
            
            let mut transport_config = quinn::TransportConfig::default();
            transport_config.max_idle_timeout(Some(
                quinn::VarInt::from_u32(crate::utils::IDLE_TIMEOUT_MS).into()
            ));
            client_config.transport_config(Arc::new(transport_config));
            
            endpoint.set_default_client_config(client_config);

            'retry_loop: loop {
                println!("[CLIENT] starting retry loop, connecting to {server_addr:?}");
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
                                continue 'retry_loop;
                            } else {
                                println!("[CLIENT] connected, starting keep alive task");
                                start_client_keep_alive_task(
                                    connection,
                                    Arc::clone(&shared_c1),
                                );
                                break 'retry_loop;
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
    /// only send messages of up to 1 mb otherwise this will likely fail
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
    let (recv_stream_opt, mut watch_rx, start_gen) = {
        let mut guard = shared.lock();
        let generation = *guard.new_connection_tx.borrow();
        (guard.recv_stream.take(), guard.new_connection_rx.clone(), generation)
    };
    
    if let Some(recv_stream) = recv_stream_opt {
        let shared_c = Arc::clone(shared);
        let result = tasker::get_tokio_handle().block_on(async move {
            // Mark the current generation as seen
            let current_gen = *watch_rx.borrow_and_update();
            
            // race condition check
            if current_gen != start_gen {
                return Err(Box::new(std::io::Error::other("Connection replaced")) as Box<dyn std::error::Error + Send + Sync>);
            }
            
            // either complete recv or detect new connection
            tasker::tokio::select! {
                recv_result = recv_messages(recv_stream, shared_c.clone(), context) => {
                    recv_result
                }
                _ = watch_rx.changed() => {
                    Err(Box::new(std::io::Error::other("Connection replaced")) as Box<dyn std::error::Error + Send + Sync>)
                }
            }
        });

        match result {
            Ok(stream) => {
                let mut guard = shared.lock();
                // avoid overwriting a new connection's stream with an old one
                if guard.recv_stream.is_none() {
                    guard.recv_stream = Some(stream);
                }
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
    
    // Take ownership of send_stream to prevent it from being replaced during send
    let mut send_stream = shared.lock().send_stream.take()
        .ok_or_else(|| Box::new(std::io::Error::other("No established connection.")) as Box<dyn std::error::Error + Send + Sync>)?;
    
    let result: Result<(), Box<dyn std::error::Error + Send + Sync>> = tasker::get_tokio_handle().block_on(async {
        send_stream
            .write_all(&cobs::encode_vec_including_sentinels(&bytes))
            .await
            .map_err(|e| Box::new(e) as Box<dyn std::error::Error + Send + Sync>)?;
        send_stream.flush().await
            .map_err(|e| Box::new(e) as Box<dyn std::error::Error + Send + Sync>)
    });
    
    // Put the stream back if still valid
    let mut guard = shared.lock();
    if guard.send_stream.is_none() {
        guard.send_stream = Some(send_stream);
    }
    
    result
}