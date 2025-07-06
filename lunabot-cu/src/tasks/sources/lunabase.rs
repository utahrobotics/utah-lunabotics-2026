use std::{fs::File, net::{IpAddr, SocketAddr}, str::FromStr, sync::Arc, time::Duration};
use crossbeam::atomic::AtomicCell;
use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuMsg, CuSrcTask, Freezable}, output_msg, prelude::*, CuError, CuResult};
use tasker::tokio::sync::{mpsc, watch};

use crate::common::{FromLunabase, FromLunabot, LunabaseConn, LunabotStage, PacketBuilder, TELEOP};

pub struct Lunabase {
    packet_builder: PacketBuilder,
    from_lunabase_rx: mpsc::UnboundedReceiver<FromLunabase>,
    max_pong_delay: u64,
    connected: LunabotConnected,
    last_pong_time: Instant,
}

impl Freezable for Lunabase{}

impl<'cl> CuSrcTask<'cl> for Lunabase {
    type Output = output_msg!('cl, Option<FromLunabase>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
        where Self: Sized 
    {
        if config.is_none() {
            return Err(CuError::new_with_cause("no config provided", std::io::Error::new(std::io::ErrorKind::Other, "no config provided")));
        }
        let config = config.unwrap();
        let lunabase_address: String = config.get("lunabase_address").unwrap();
        let lunabase_address = IpAddr::from_str(&lunabase_address).unwrap();
        let max_pong_delay = config.get("max_pong_delay_ms").unwrap_or(default_max_pong_delay_ms());

        let lunabot_stage = Arc::new(AtomicCell::new(LunabotStage::SoftStop));
        let (packet_builder, from_lunabase_rx, connected) = create_packet_builder(
            Some(SocketAddr::new(lunabase_address, TELEOP)),
            lunabot_stage.clone(),
            max_pong_delay,
        );

        Ok(Self {
            packet_builder,
            from_lunabase_rx,
            max_pong_delay,
            connected,
            last_pong_time: Instant::now(),
        })
    }

    fn process(&mut self, _: &RobotClock, output: Self::Output) -> CuResult<()> {
        if let Ok(from_lunabase) = self.from_lunabase_rx.try_recv() {
            if from_lunabase == FromLunabase::Pong {
                self.last_pong_time = Instant::now();
            }
            output.set_payload(Some(from_lunabase));
        }
        if self.last_pong_time.elapsed() > Duration::from_millis(self.max_pong_delay) {
            return Err(CuError::new_with_cause("no pong received from lunabase", std::io::Error::new(std::io::ErrorKind::Other, "no pong received from lunabase")));
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
