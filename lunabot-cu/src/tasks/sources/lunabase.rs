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

#[cfg(feature = "production")]
use crate::comms::{
    LunabaseConn, LunabotConnected, PacketBuilder, create_packet_builder, default_max_pong_delay_ms,
};
use common::{FromLunabase, LUNABOT_STAGE, LunabotStage, ports::TELEOP};

pub struct Lunabase {
    from_lunabase_rx: mpsc::UnboundedReceiver<FromLunabase>,
    #[cfg(feature = "production")]
    connected: LunabotConnected,
    message_buffer: VecDeque<FromLunabase>,
}

impl Freezable for Lunabase {}

impl CuSrcTask for Lunabase {
    type Output<'m> = output_msg!(Option<FromLunabase>);

    #[cfg(any(feature = "sim", feature = "resim"))]
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        let (_, rx) = mpsc::unbounded_channel();
        Ok(Self {
            from_lunabase_rx: rx,
            message_buffer: VecDeque::new(),
        })
    }

    #[cfg(feature = "production")]
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
        #[cfg(feature = "production")]
        if !self.connected.is_connected() {
            self.message_buffer.clear();
            self.message_buffer.push_back(FromLunabase::Disconnect);
            status = Err(CuError::new_with_cause(
                "lunabase not connected",
                std::io::Error::other("lunabase disconnected"),
            ));
        } else {
            status = Ok(())
        }

        #[cfg(any(feature = "sim", feature = "resim"))]
        {
            status = Ok(());
        }

        output.set_payload(self.message_buffer.pop_front());
        output.metadata.process_time.start = clock.now().into();

        status
    }
}
