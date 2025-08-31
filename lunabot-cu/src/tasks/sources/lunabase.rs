use crossbeam::atomic::AtomicCell;
use cu29::{
    CuError, CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuMsg, CuSrcTask, Freezable},
    output_msg,
    prelude::*,
};
use std::{
    collections::VecDeque,
    net::{IpAddr, SocketAddr},
    str::FromStr,
    sync::Arc,
    time::Duration,
};
use tasker::tokio::sync::mpsc::error::TryRecvError;
use tasker::tokio::sync::{mpsc, watch};

use crate::comms::{LunabaseConn, PacketBuilder, TELEOP};
use common::{FromLunabase, LUNABOT_STAGE, LunabotStage};

pub struct Lunabase {
    from_lunabase_rx: mpsc::UnboundedReceiver<FromLunabase>,
    connected: LunabotConnected,
    message_buffer: VecDeque<FromLunabase>,
}

impl Freezable for Lunabase {}

impl CuSrcTask for Lunabase {
    type Output<'m> = output_msg!(Option<FromLunabase>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        // Configuration is optional: if no address is provided, we rely on autodiscovery
        let (lunabase_address_opt, max_pong_delay): (Option<SocketAddr>, u64) =
            if let Some(cfg) = config {
                let max_delay = cfg
                    .get("max_pong_delay_ms")
                    .unwrap_or(default_max_pong_delay_ms());

                // "lunabase_address" parameter is optional. If missing we fall back to None for auto-discover.
                let addr_opt: Option<SocketAddr> = cfg
                    .get::<String>("lunabase_address")
                    .and_then(|s| IpAddr::from_str(&s).ok())
                    .map(|ip| SocketAddr::new(ip, TELEOP));

                (addr_opt, max_delay)
            } else {
                (None, default_max_pong_delay_ms())
            };

        let lunabot_stage = LUNABOT_STAGE.clone();
        let (_, from_lunabase_rx, connected) =
            create_packet_builder(lunabase_address_opt, lunabot_stage.clone(), max_pong_delay);

        Ok(Self {
            from_lunabase_rx,
            connected,
            message_buffer: VecDeque::new(),
        })
    }

    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        loop {
            match self.from_lunabase_rx.try_recv() {
                Ok(msg) => {
                    self.message_buffer.push_back(msg);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    return Err(CuError::from("Lunabase channel disconnected"));
                }
            }
        }
        let status;
        if !self.connected.is_connected() {
            if LUNABOT_STAGE.load() != LunabotStage::SoftStop {
                self.message_buffer.push_back(FromLunabase::SoftStop);
            }
            status = Err(CuError::new_with_cause(
                "lunabase not connected",
                std::io::Error::other("lunabase disconnected"),
            ));
        } else {
            status = Ok(())
        }

        // Forward message to downstream tasks and keep the global stage in sync so that
        // Ping packets always advertise the correct mode, even if the AI-side SetStage
        // packet was lost in transport.
        if let Some(ref msg) = self.message_buffer.pop_front() {
            match msg {
                FromLunabase::SoftStop => LUNABOT_STAGE.store(LunabotStage::SoftStop),
                FromLunabase::ContinueMission => LUNABOT_STAGE.store(LunabotStage::TeleOp),
                FromLunabase::Navigate(_) | FromLunabase::DigDump(_) => {
                    LUNABOT_STAGE.store(LunabotStage::Autonomy)
                }
                _ => {}
            }

            output.set_payload(Some(*msg));
        } else {
            output.clear_payload();
        }
        status
    }
}

pub fn default_max_pong_delay_ms() -> u64 {
    1500
}

#[derive(Clone)]
struct LunabotConnected {
    connected: watch::Receiver<bool>,
}

impl LunabotConnected {
    fn is_connected(&self) -> bool {
        *self.connected.borrow()
    }
}

fn create_packet_builder(
    lunabase_address: Option<SocketAddr>,
    lunabot_stage: Arc<AtomicCell<LunabotStage>>,
    max_pong_delay_ms: u64,
) -> (
    PacketBuilder,
    mpsc::UnboundedReceiver<FromLunabase>,
    LunabotConnected,
) {
    let (from_lunabase_tx, from_lunabase_rx) = mpsc::unbounded_channel();
    let mut bitcode_buffer = bitcode::Buffer::new();
    let (pinged_tx, pinged_rx) = std::sync::mpsc::channel::<()>();

    let packet_builder = LunabaseConn {
        lunabase_address,
        on_msg: move |bytes: &[u8]| match bitcode_buffer.decode(bytes) {
            Ok(msg) => {
                if msg == FromLunabase::Pong {
                    let _ = pinged_tx.send(());
                } else {
                    let _ = from_lunabase_tx.send(msg);
                }
                true
            }
            Err(e) => {
                error!("Failed to decode from lunabase: {}", e.to_string());
                false
            }
        },
        lunabot_stage,
    }
    .connect_to_lunabase();

    let (connected_tx, connected_rx) = watch::channel(false);

    std::thread::spawn(move || {
        loop {
            match pinged_rx.recv_timeout(Duration::from_millis(max_pong_delay_ms)) {
                Ok(()) => {
                    let _ = connected_tx.send(true);
                }
                Err(_) => {
                    let _ = connected_tx.send(false);
                }
            }
        }
    });

    let connected = LunabotConnected {
        connected: connected_rx,
    };

    (packet_builder, from_lunabase_rx, connected)
}
