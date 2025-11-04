use std::collections::HashMap;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use bincode::error::DecodeError;
use common::{FromLunabase, FromLunabot, LunabotStage, Steering};
use crossbeam_channel::{Receiver, Sender};
use godot::prelude::*;
use quic::QuicClient;

struct LunabaseExtension;

#[gdextension]
unsafe impl ExtensionLibrary for LunabaseExtension {}

#[derive(GodotClass)]
#[class(base=Node)]
struct LunabaseConnection {
    base: Base<Node>,

    #[export]
    default_address: GString,

    // <outgoing type, incoming type>
    client: Option<QuicClient<FromLunabase, FromLunabot>>,
    last_packet_time: Instant,
    current_stage: LunabotStage,
    recv_thread: Option<JoinHandle<()>>,

    errored_tasks: Arc<Mutex<HashMap<String, String>>>,
}

#[godot_api]
impl INode for LunabaseConnection {
    fn init(base: Base<Node>) -> Self {
        LunabaseConnection {
            errored_tasks: Arc::new(Mutex::new(HashMap::new())),
            base,
            default_address: GString::from("127.0.0.1"),
            client: None,
            recv_thread: None,
            last_packet_time: Instant::now(),
            current_stage: LunabotStage::SoftStop,
        }
    }

    fn ready(&mut self) {
        let address = self.default_address.to_string();
        self.connect_to_address(address);
    }

    fn process(&mut self, _delta: f64) {
        if let Some(ref client) = self.client {
            if let Some(encoded_stage) = client.get_last_keep_alive_msg() {
                self.last_packet_time = Instant::now();
                self.base_mut().emit_signal("packet_received", &[]);

                let reported_lunabot_stage: Result<(LunabotStage, usize), DecodeError> =
                    bincode::borrow_decode_from_slice(&encoded_stage, bincode::config::standard());

                if let Ok((stage, _)) = reported_lunabot_stage {
                    if stage != self.current_stage {
                        self.current_stage = stage;
                        self.base_mut()
                            .emit_signal("stage_changed", &[Variant::from(stage as i32)]);
                    }
                }
            }
        }
    }
}

#[godot_api]
impl LunabaseConnection {
    #[signal]
    fn packet_received();

    #[signal]
    fn stage_changed(stage: i32);

    fn connect_to_address(&mut self, address_str: String) {
        let lunabot_address = {
            if let Ok(addr) = address_str.parse::<Ipv4Addr>() {
                godot_warn!("Connecting to: {address_str}");
                Some(addr)
            } else {
                godot_error!("Failed to parse address: {address_str}");
                return;
            }
        };

        if let Some(addr) = lunabot_address {
            let socket_addr = SocketAddr::V4(SocketAddrV4::new(addr, common::ports::TELEOP));

            match QuicClient::connect(socket_addr) {
                Ok(client) => {
                    let cleint_c = client.clone();
                    let errored_tasks = self.errored_tasks.clone();
                    let recv_thread = std::thread::spawn(move || {
                        loop {
                            match cleint_c.recv() {
                                Ok(msg) => match msg {
                                    FromLunabot::RobotIsometry { .. } => {}
                                    FromLunabot::ArmAngles { .. } => {}
                                    FromLunabot::ErroredTasks(hash_map) => {
                                        if let Ok(mut guard) = errored_tasks.lock() {
                                            *guard = hash_map;
                                        }
                                    }
                                },
                                Err(e) => {
                                    eprintln!("Failed to receive message from server: {}", e);
                                    std::thread::sleep(Duration::from_millis(5));
                                }
                            }
                        }
                    });
                    self.client = Some(client);
                    self.recv_thread = Some(recv_thread);
                    godot_print!("Successfully connected to {address_str}");
                }
                Err(e) => {
                    godot_error!("Failed to create client: {}", e);
                }
            }
        }
    }

    #[func]
    fn reconnect(&mut self, address: GString) {
        let address_str = address.to_string();
        self.connect_to_address(address_str);
    }

    #[func]
    fn get_ms_since_last_packet(&self) -> i64 {
        if let Some(ref client) = self.client {
            if let Some(time_since) = client.time_since_last_keep_alive() {
                time_since.as_millis() as i64
            } else {
                99999 // lmao
            }
        } else {
            99999
        }
    }

    #[func]
    fn get_lunabot_stage(&self) -> i32 {
        self.current_stage as i32
    }

    #[func]
    fn execute_steering(&self, left: f64, right: f64) {
        if let Some(ref client) = self.client {
            match client.send(FromLunabase::Steering(Steering::new(left, right, 1.0))) {
                Ok(_) => {}
                Err(e) => {
                    godot_warn!("Failed to send steering packet: {e}");
                }
            }
        } else {
            godot_warn!("Cannot send steering: not connected");
        }
    }

    #[func]
    fn send_lift_actuators(&self, speed: f64) {
        if let Some(ref client) = self.client {
            match client.send(FromLunabase::set_lift_actuator(speed)) {
                Ok(_) => {}
                Err(e) => {
                    godot_warn!("Failed to send lift actuator packet: {e}");
                }
            }
        } else {
            godot_warn!("Cannot send lift actuators: not connected");
        }
    }

    #[func]
    fn send_bucket_actuators(&self, speed: f64) {
        if let Some(ref client) = self.client {
            match client.send(FromLunabase::set_bucket_actuator(speed)) {
                Ok(_) => {}
                Err(e) => {
                    godot_warn!("Failed to send bucket actuator packet: {e}");
                }
            }
        } else {
            godot_warn!("Cannot send bucket actuators: not connected");
        }
    }

    #[func]
    fn send_soft_stop(&self) {
        if let Some(ref client) = self.client {
            match client.send(FromLunabase::SoftStop) {
                Ok(_) => {
                    godot_print!("Sent SoftStop command");
                }
                Err(e) => {
                    godot_warn!("Failed to send soft stop packet: {e}");
                }
            }
        } else {
            godot_warn!("Cannot send soft stop: not connected");
        }
    }

    #[func]
    fn send_continue_mission(&self) {
        if let Some(ref client) = self.client {
            match client.send(FromLunabase::ContinueMission) {
                Ok(_) => {
                    godot_print!("Sent ContinueMission command");
                }
                Err(e) => {
                    godot_warn!("Failed to send continue mission packet: {e}");
                }
            }
        } else {
            godot_warn!("Cannot send continue mission: not connected");
        }
    }

    #[func]
    fn send_start_autonomy(&self) {
        if let Some(ref client) = self.client {
            match client.send(FromLunabase::Navigate((0.0, 0.0))) {
                Ok(_) => {
                    godot_print!("Sent Navigate command");
                }
                Err(e) => {
                    godot_warn!("Cannot send start Autonomy command: {e}");
                }
            }
        }
    }

    #[func]
    fn get_errored_tasks(&self) -> Dictionary {
        let mut dict = Dictionary::new();
        if let Ok(guard) = self.errored_tasks.lock() {
            for (task_name, error_msg) in guard.iter() {
                dict.set(task_name.as_str(), error_msg.as_str());
            }
        }
        dict
    }
}
