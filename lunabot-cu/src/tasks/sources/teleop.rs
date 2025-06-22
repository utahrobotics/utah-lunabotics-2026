use std::{fs::File, net::SocketAddr, sync::Arc, time::Duration};
use crossbeam::atomic::AtomicCell;
use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuMsg, CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use tasker::tokio::sync::{mpsc, watch};

use crate::common::{FromLunabase, FromLunabot, LunabaseConn, LunabotStage, PacketBuilder};

pub struct Teleop {
    packet_builder: PacketBuilder,
    from_lunabase_rx: mpsc::UnboundedReceiver<FromLunabase>,
    max_pong_delay: u64
}

impl Freezable for Teleop{}

impl<'cl> CuSrcTask<'cl> for Teleop {
    type Output = output_msg!('cl, Option<FromLunabase>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
        where Self: Sized 
    {
        // create the packet builder using the helper function
        todo!()
    }

    fn process(&mut self, clock: &RobotClock, new_msg: Self::Output) -> CuResult<()> {
        // this process loop runs once per tick
        todo!()
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
