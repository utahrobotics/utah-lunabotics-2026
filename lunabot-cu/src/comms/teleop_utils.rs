use std::{
    sync::{Arc, Mutex, OnceLock},
    time::{Duration, Instant},
};

use common::{
    COMMAND_STREAM_ID, ERROR_STREAM_ID, POSE_STREAM_ID, FromLunabase, FromLunabot, LunabotStage,
    ports::TELEOP,
};
use crossbeam::atomic::AtomicCell;
use quic::{
    StreamHandle,
    server::{QuicServer, ServerConnection},
};
use tasker::{get_tokio_handle, tokio};

/// Timestamp of the last received message from Lunabase (excluding keep alive packets)
static LAST_SEEN_TIMESTAMP_COMMAND: OnceLock<AtomicCell<Instant>> = OnceLock::new();

/// Provides a non-blocking interface to the QUIC server.
///
/// Spawns background tokio tasks that accept connections, receive messages,
/// and send messages. New connections automatically replace old ones.
///
/// Uses three multiplexed QUIC streams:
/// - Command stream (ID 0): receives `FromLunabase` commands
/// - Pose stream (ID 1): sends high-frequency pose data (`FromLunabot`)
/// - Error stream (ID 2): sends low-frequency error messages (`FromLunabot::ErroredTasks`)
pub struct LunabaseConnection {
    incoming: crossbeam_channel::Receiver<FromLunabase>,
    outgoing_pose_stream: tokio::sync::mpsc::UnboundedSender<FromLunabot>,
    outgoing_error_stream: tokio::sync::mpsc::UnboundedSender<FromLunabot>,
    shared_conn: Arc<Mutex<Option<ServerConnection<LunabotStage>>>>,
}

impl LunabaseConnection {
    pub fn new(
        keep_alive_interval: Duration,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let _guard = get_tokio_handle().enter();
        let server = QuicServer::new(TELEOP)?;

        LAST_SEEN_TIMESTAMP_COMMAND.get_or_init(|| AtomicCell::new(Instant::now()));

        let (tx_incoming_command_stream, rx_incoming_command_stream) =
            crossbeam_channel::unbounded::<FromLunabase>();
        let (tx_outgoing_pose_stream, rx_outgoing_pose_stream) =
            tokio::sync::mpsc::unbounded_channel::<FromLunabot>();
        let (tx_outgoing_error_stream, rx_outgoing_error_stream) =
            tokio::sync::mpsc::unbounded_channel::<FromLunabot>();

        let shared_conn: Arc<Mutex<Option<ServerConnection<LunabotStage>>>> =
            Arc::new(Mutex::new(None));
        let shared_conn_clone = Arc::clone(&shared_conn);

        get_tokio_handle().spawn(accept_loop(
            server,
            keep_alive_interval,
            shared_conn_clone,
            tx_incoming_command_stream,
            rx_outgoing_pose_stream,
            rx_outgoing_error_stream,
        ));

        Ok(Self {
            incoming: rx_incoming_command_stream,
            outgoing_pose_stream: tx_outgoing_pose_stream,
            outgoing_error_stream: tx_outgoing_error_stream,
            shared_conn,
        })
    }

    /// Try to receive a command from Lunabase without blocking.
    pub fn try_recv(&self) -> Option<FromLunabase> {
        self.incoming.try_recv().ok()
    }

    /// Try to send a message to Lunabase without blocking.
    /// Routes `ErroredTasks` to the error stream and everything else to the pose stream.
    pub fn try_send(
        &self,
        msg: FromLunabot,
    ) -> Result<(), tokio::sync::mpsc::error::SendError<FromLunabot>> {
        match &msg {
            FromLunabot::ErroredTasks(_) => self.outgoing_error_stream.send(msg),
            _ => self.outgoing_pose_stream.send(msg),
        }
    }

    /// Check if the connection is alive based on the last seen timestamp.
    pub fn is_alive(&self, timeout: Duration) -> bool {
        if let Some(timestamp) = self.time_since_last_seen() {
            timestamp < timeout
        } else {
            false
        }
    }

    /// Set the keep-alive message on the current connection (if any).
    pub fn set_keep_alive_msg(&self, msg: LunabotStage) {
        if let Some(conn) = self.shared_conn.lock().unwrap().as_ref() {
            conn.keep_alive.set_msg(msg);
        }
    }

    /// Get the time since last activity (message or keep-alive).
    pub fn time_since_last_seen(&self) -> Option<Duration> {
        if let Some(conn) = self.shared_conn.lock().unwrap().as_ref() {
            return conn.keep_alive.time_since_last();
        } else {
            LAST_SEEN_TIMESTAMP_COMMAND
                .get()
                .map(|ts| ts.load().elapsed())
        }
    }
}

async fn accept_loop(
    server: QuicServer,
    keep_alive_interval: Duration,
    shared_conn: Arc<Mutex<Option<ServerConnection<LunabotStage>>>>,
    tx_incoming: crossbeam_channel::Sender<FromLunabase>,
    rx_outgoing_pose: tokio::sync::mpsc::UnboundedReceiver<FromLunabot>,
    rx_outgoing_error: tokio::sync::mpsc::UnboundedReceiver<FromLunabot>,
) {
    // Wrap receivers in Arc<Mutex> so they survive across reconnections.
    // When a send task is aborted on reconnect, the mutex is released and the
    // new task can acquire it. Queued messages are preserved.
    let rx_pose = Arc::new(tokio::sync::Mutex::new(rx_outgoing_pose));
    let rx_error = Arc::new(tokio::sync::Mutex::new(rx_outgoing_error));

    loop {
        let connection = match server
            .accept(keep_alive_interval, LunabotStage::SoftStop)
            .await
        {
            Ok(c) => c,
            Err(e) => {
                eprintln!("[LunabaseConnection] Accept error: {e}");
                tokio::time::sleep(Duration::from_millis(100)).await;
                continue;
            }
        };

        // Accept all 3 streams sequentially.
        // The quic crate buffers non-matching stream IDs, so
        // the client can open them in any order without deadlock.
        let command_stream = match connection
            .accept_bi::<FromLunabot, FromLunabase, { COMMAND_STREAM_ID }>()
            .await
        {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[LunabaseConnection] Failed to accept command stream: {e}");
                continue;
            }
        };

        let pose_stream = match connection
            .accept_bi::<FromLunabot, FromLunabase, { POSE_STREAM_ID }>()
            .await
        {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[LunabaseConnection] Failed to accept pose stream: {e}");
                continue;
            }
        };

        let error_stream = match connection
            .accept_bi::<FromLunabot, FromLunabase, { ERROR_STREAM_ID }>()
            .await
        {
            Ok(s) => s,
            Err(e) => {
                eprintln!("[LunabaseConnection] Failed to accept error stream: {e}");
                continue;
            }
        };

        let conn_for_watch = connection.quinn().clone();
        *shared_conn.lock().unwrap() = Some(connection);

        if let Some(ts) = LAST_SEEN_TIMESTAMP_COMMAND.get() {
            ts.store(Instant::now());
        }

        let recv_handle = tokio::spawn(recv_loop(command_stream, tx_incoming.clone()));
        let send_pose_handle =
            tokio::spawn(send_loop(pose_stream, Arc::clone(&rx_pose)));
        let send_error_handle =
            tokio::spawn(send_loop(error_stream, Arc::clone(&rx_error)));
        let recv_abort = recv_handle.abort_handle();
        let send_pose_abort = send_pose_handle.abort_handle();
        let send_error_abort = send_error_handle.abort_handle();

        // watchdog
        let conn_closed = conn_for_watch.closed();
        tokio::select! {
            _ = recv_handle => {
                eprintln!("[LunabaseConnection] command recv task died");
            }
            _ = send_pose_handle => {
                eprintln!("[LunabaseConnection] pose send task died");
            }
            _ = send_error_handle => {
                eprintln!("[LunabaseConnection] error send task died");
            }
            _ = conn_closed => {
                eprintln!("[LunabaseConnection] QUIC connection closed");
            }
        }

        recv_abort.abort();
        send_pose_abort.abort();
        send_error_abort.abort();

        conn_for_watch.close(2u32.into(), b"stream died");
        *shared_conn.lock().unwrap() = None;
        eprintln!("[LunabaseConnection] Connection torn down, ready to accept again");
    }
}

async fn recv_loop(
    stream: StreamHandle<FromLunabot, FromLunabase>,
    tx: crossbeam_channel::Sender<FromLunabase>,
) {
    loop {
        match stream.recv().await {
            Ok(msg) => {
                if let Some(ts) = LAST_SEEN_TIMESTAMP_COMMAND.get() {
                    ts.store(Instant::now());
                }
                if tx.len() > 5 {
                    eprintln!(
                        "[LunabaseConnection] Warning: incoming queue has {} messages",
                        tx.len()
                    );
                }
                if tx.send(msg).is_err() {
                    break;
                }
            }
            Err(e) => {
                eprintln!("[LunabaseConnection] Recv stream error: {e}");
                break;
            }
        }
    }
}

async fn send_loop(
    stream: StreamHandle<FromLunabot, FromLunabase>,
    rx: Arc<tokio::sync::Mutex<tokio::sync::mpsc::UnboundedReceiver<FromLunabot>>>,
) {
    let mut guard = rx.lock().await;
    while let Some(msg) = guard.recv().await {
        if let Err(e) = stream.send(&msg).await {
            eprintln!("[LunabaseConnection] Send stream error: {e}");
            break;
        }
    }
}
