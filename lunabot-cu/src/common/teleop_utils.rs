use std::{
    net::{Ipv4Addr, SocketAddr, SocketAddrV4},
    ops::Deref,
    sync::Arc,
    time::{Duration, Instant},
};

use cakap2::{packet::Action, Event, PeerStateMachine, RecommendedAction};
use crossbeam::atomic::AtomicCell;
use cu29::prelude::*;
use tasker::get_tokio_handle;
use tasker::tokio::{self, net::UdpSocket, sync::mpsc};

use crate::common::{FromLunabot, LunabotStage, TELEOP};

#[derive(Clone)]
pub struct PacketBuilder {
    builder: cakap2::packet::PacketBuilder,
    #[allow(dead_code)]
    packet_tx: mpsc::UnboundedSender<Action>,
}

impl Deref for PacketBuilder {
    type Target = cakap2::packet::PacketBuilder;

    fn deref(&self) -> &Self::Target {
        &self.builder
    }
}

impl PacketBuilder {
    #[allow(dead_code)]
    pub fn send_packet(&self, packet: Action) {
        let _ = self.packet_tx.send(packet);
    }
}

pub struct LunabaseConn<F> {
    pub lunabase_address: Option<SocketAddr>,
    pub on_msg: F,
    pub lunabot_stage: Arc<AtomicCell<LunabotStage>>,
}

impl<F: FnMut(&[u8]) -> bool + Send + 'static> LunabaseConn<F> {
    /// Connect to the lunabase and return a [`PacketBuilder`] to send packets to the lunabase.
    ///
    /// The `on_msg` closure is called whenever a message is received from the lunabase, and must
    /// return `true` if the message was successfully parsed, and `false` otherwise.
    pub fn connect_to_lunabase(mut self) -> PacketBuilder {
        let mut cakap_sm = PeerStateMachine::new(Duration::from_millis(150), 1024, 1400);
        let packet_builder = cakap_sm.get_packet_builder();
        let (packet_tx, mut packet_rx) = mpsc::unbounded_channel();

        get_tokio_handle().spawn(async move {
            let udp = loop {
                let udp = match UdpSocket::bind(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, TELEOP)).await {
                    Ok(x) => x,
                    Err(e) => {
                        error!("Failed to bind to lunabase address: {}", e.to_string());
                        tokio::time::sleep(Duration::from_secs(1)).await;
                        continue;
                    }
                };
                // if let Err(e) = udp.connect(self.lunabase_address).await {
                //     error!("Failed to connect to lunabase: {e}");
                //     tokio::time::sleep(Duration::from_secs(1)).await;
                //     continue;
                // }
                break udp;
            };

            let mut action: RecommendedAction<'_, '_> = cakap_sm.send_reconnection_msg(Instant::now()).0;
            let mut wait_for: Option<Duration>;

            macro_rules! send {
                ($data: expr) => {{
                    loop {
                        if let Some(addr) = self.lunabase_address {
                            if let Err(e) = udp.send_to($data, addr).await {
                                if e.kind() == std::io::ErrorKind::ConnectionRefused {
                                    if addr.ip().is_loopback() {
                                        continue;
                                    }
                                }
                                error!("Failed to send data to lunabase: {}", e.to_string());
                                continue;
                            }
                        }
                        action = cakap_sm.poll(Event::NoEvent, Instant::now());
                        break;
                    }
                }};
            }

            let mut buf= [0u8; 1408];
            macro_rules! handle {
                () => {
                    loop {
                        match action {
                            RecommendedAction::WaitForData => {
                                wait_for = None;
                                break;
                            }
                            RecommendedAction::WaitForDuration(duration) => {
                                wait_for = Some(duration);
                                break;
                            }
                            RecommendedAction::HandleError(cakap_error) => {
                                error!("{}", cakap_error.to_string());
                                action = cakap_sm.poll(Event::NoEvent, Instant::now());
                            }
                            RecommendedAction::HandleData(received) => {
                                (self.on_msg)(&received);
                                action = cakap_sm.poll(Event::NoEvent, Instant::now());
                            }
                            RecommendedAction::HandleDataAndSend { received, to_send } =>  if (self.on_msg)(&received) {
                                send!(&to_send);
                            }
                            RecommendedAction::SendData(hot_packet) => {
                                send!(&hot_packet);
                            }
                        }
                    }
                }
            }
            handle!();
            let mut bitcode_buffer = bitcode::Buffer::new();
            let mut ping_at = tokio::time::Instant::now();

            loop {
                tokio::select! {
                    _ = tokio::time::sleep_until(ping_at) => {
                        let bytes = bitcode_buffer.encode(&FromLunabot::Ping(self.lunabot_stage.load()));
                        let packet = cakap_sm.get_packet_builder().new_unreliable(bytes.to_vec().into()).unwrap();
                        action = cakap_sm.poll(Event::Action(Action::SendUnreliable(packet)), Instant::now());
                        handle!();
                        ping_at = tokio::time::Instant::now() + Duration::from_millis(800);
                        continue;
                    }
                    _ = async {
                        if let Some(duration) = wait_for {
                            tokio::time::sleep(duration).await;
                        } else {
                            std::future::pending::<()>().await;
                        }
                    } => {
                        action = cakap_sm.poll(Event::NoEvent, Instant::now());
                    }
                    packet = async {
                        if let Some(packet) = packet_rx.recv().await {
                            packet
                        } else {
                            warning!("Packet channel closed");
                            std::future::pending().await
                        }
                    } => {
                        action = cakap_sm.poll(Event::Action(packet), Instant::now());
                    }
                    result = udp.recv_from(&mut buf) => {
                        let (n, addr) = match result {
                            Ok(x) => x,
                            Err(e) => {
                                if e.kind() == std::io::ErrorKind::ConnectionRefused {
                                    if let Some(addr) = self.lunabase_address {
                                        if addr.ip().is_loopback() {
                                            continue;
                                        }
                                    }
                                }
                                error!("Failed to receive data from lunabase: {}", e.to_string());
                                continue;
                            }
                        };
                        // if addr.port() != common::ports::TELEOP {
                        //     error!("Received data from unexpected address on teleop: {addr}");
                        //     continue;
                        // }
                        self.lunabase_address = Some(addr);
                        action = cakap_sm.poll(Event::IncomingData(&buf[..n]), Instant::now());
                    }
                }
                handle!();
            }
        });

        PacketBuilder {
            builder: packet_builder,
            packet_tx,
        }
    }
}