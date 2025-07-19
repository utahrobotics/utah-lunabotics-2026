use std::{fs::File, net::{IpAddr, SocketAddr}, str::FromStr, sync::Arc, time::Duration};
use crossbeam::atomic::AtomicCell;
use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuMsg, CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use tasker::tokio::sync::{mpsc, watch};
use tasker::tokio::sync::mpsc::error::TryRecvError;

use common::{FromLunabase, FromLunabot, LunabotStage, LUNABOT_STAGE};
use crate::comms::{LunabaseConn, PacketBuilder, TELEOP};
use serde::Serialize;

pub struct Lunabase {
    packet_builder: PacketBuilder,
    from_lunabase_rx: mpsc::UnboundedReceiver<FromLunabase>,
    max_pong_delay: u64,
    connected: LunabotConnected,
}

impl Freezable for Lunabase{}

impl<'cl> CuSrcTask<'cl> for Lunabase {
    type Output = output_msg!('cl, Option<FromLunabase>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
        where Self: Sized 
    {
        // Configuration is optional: if no address is provided, we rely on autodiscovery
        let (lunabase_address_opt, max_pong_delay): (Option<SocketAddr>, u64) = if let Some(cfg) = config {
            let max_delay = cfg.get("max_pong_delay_ms").unwrap_or(default_max_pong_delay_ms());

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
        let (packet_builder, from_lunabase_rx, connected) = create_packet_builder(
            lunabase_address_opt,
            lunabot_stage.clone(),
            max_pong_delay,
        );

        Ok(Self {
            packet_builder,
            from_lunabase_rx,
            max_pong_delay,
            connected,
        })
    }

    fn process(&mut self, clock: &RobotClock, output: Self::Output) -> CuResult<()> {        
        // Collect the latest message from lunabase without blocking. The loop drains the
        // receiver so we only forward the most recent command each cycle – this keeps the
        // process function fast while ensuring we never fall behind.
        let mut latest_msg: Option<FromLunabase> = None;
        loop {
            match self.from_lunabase_rx.try_recv() {
                Ok(msg) => {
                    latest_msg = Some(msg);
                }
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    return Err(CuError::from("Lunabase channel disconnected"));
                }
            }
        }
        if !*self.connected.connected.borrow() {
            latest_msg = Some(FromLunabase::SoftStop);
        }

        // Forward message to downstream tasks and keep the global stage in sync so that
        // Ping packets always advertise the correct mode, even if the AI-side SetStage
        // packet was lost in transport.
        if let Some(ref msg) = latest_msg {
            // Update global stage heuristically based on operator commands.
            match msg {
                FromLunabase::SoftStop => LUNABOT_STAGE.store(LunabotStage::SoftStop),
                FromLunabase::ContinueMission => LUNABOT_STAGE.store(LunabotStage::TeleOp),
                // A navigate / dig-dump command implies autonomous mode.
                FromLunabase::Navigate(_) | FromLunabase::DigDump(_) => {
                    LUNABOT_STAGE.store(LunabotStage::Autonomy)
                }
                FromLunabase::LiftActuators(cmd) => {
                    info!("Lunabase: lift actuators command {}", cmd.to_string());
                }
                _ => {}
            }

            output.set_payload(Some(*msg));
        } else {
            // No new message this cycle – clear any previous payload.
            output.clear_payload();
        }
        Ok(())
    }
}

pub fn default_max_pong_delay_ms() -> u64 {
    1500
}

fn log_teleop_messages() -> CuResult<()> {
    if let Err(e) = File::create("from_lunabase.txt")
        .map(|f| FromLunabase::write_code_sheet(f))
        .flatten()
    {
        return Err(CuError::new_with_cause("failed to write code sheet for FromLunabase", e));
    }
    if let Err(e) = File::create("from_lunabot.txt")
        .map(|f| FromLunabot::write_code_sheet(f))
        .flatten()
    {
        return Err(CuError::new_with_cause("failed to write code sheet for FromLunabot", e));
    }

    Ok(())
}

#[derive(Clone)]
struct LunabotConnected {
    connected: watch::Receiver<bool>,
}

impl LunabotConnected {
    // fn is_connected(&self) -> bool {
    //     *self.connected.borrow()
    // }

    async fn wait_disconnect(&mut self) {
        let _ = self.connected.wait_for(|&x| !x).await;
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

    std::thread::spawn(move || loop {
        match pinged_rx.recv_timeout(Duration::from_millis(max_pong_delay_ms)) {
            Ok(()) => {
                let _ = connected_tx.send(true);
            }
            Err(_) => {
                let _ = connected_tx.send(false);
            }
        }
    });

    let connected = LunabotConnected {
        connected: connected_rx,
    };

    (packet_builder, from_lunabase_rx, connected)
}
