use std::collections::HashMap;
use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};
use std::ops::Deref;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use common::{
    BT_STATUS_STREAM_ID, COMMAND_STREAM_ID, ERROR_STREAM_ID, FromLunabase, FromLunabot, LunabotStage, POSE_STREAM_ID, Steering
};
use godot::prelude::*;
use quic::KeepAliveState;
use quic::client::QuicClient;
use tasker::{get_tokio_handle, tokio};

struct LunabaseExtension;

#[gdextension]
unsafe impl ExtensionLibrary for LunabaseExtension {}

#[derive(GodotClass)]
#[class(base=Node)]
struct LunabaseConnection {
    base: Base<Node>,

    #[export]
    default_address: GString,

    outgoing: Arc<Mutex<Option<tokio::sync::mpsc::UnboundedSender<FromLunabase>>>>,
    ka_state: Arc<Mutex<Option<Arc<Mutex<KeepAliveState<LunabotStage>>>>>>,
    /// Abort handle for the currently-active background QUIC task. Aborted on
    /// reconnect.
    bg_task: Option<tokio::task::AbortHandle>,
    last_packet_time: Option<Instant>,
    current_stage: LunabotStage,

    errored_tasks: Arc<Mutex<HashMap<String, String>>>,
    bt_status_msg: Arc<Mutex<String>>,
    current_weight: f64,
    global_position: Arc<Mutex<[f32; 3]>>,
    orientation: Arc<Mutex<[f32; 4]>>,

    first_connect: bool,
    apriltags_enabled: bool,
}

#[godot_api]
impl INode for LunabaseConnection {
    fn init(base: Base<Node>) -> Self {
        tasker::get_tokio_handle();
        LunabaseConnection {
            base,
            default_address: GString::from("127.0.0.1"),
            outgoing: Arc::new(Mutex::new(None)),
            ka_state: Arc::new(Mutex::new(None)),
            bg_task: None,
            last_packet_time: None,
            current_stage: LunabotStage::SoftStop,
            current_weight: 1250.0,
            errored_tasks: Arc::new(Mutex::new(HashMap::new())),
            global_position: Arc::new(Mutex::new([0.0; 3])),
            orientation: Arc::new(Mutex::new([0.0; 4])),
            first_connect: true,
            bt_status_msg: Arc::new(Mutex::new(String::new())),
            apriltags_enabled: true,
        }
    }

    fn ready(&mut self) {
        let address = self.default_address.to_string();
        self.connect_to_address(address);
    }

    fn process(&mut self, _delta: f64) {
        let mut send_packet_rx_signal = false;
        let mut new_state = None;

        {
            let guard = self.ka_state.lock().unwrap();
            if let Some(ref state_arc) = *guard {
                let state = state_arc.lock().unwrap();
                if let Some(stage) = state.last_msg() {
                    if let Some(elapsed) = state.time_since_last() {
                        if elapsed.as_millis() < 500 {
                            self.last_packet_time = Some(Instant::now());
                            send_packet_rx_signal = true;
                            if stage != self.current_stage {
                                self.current_stage = stage;
                                new_state = Some(stage);
                            }
                        }
                    }
                }
            } else if self.last_packet_time.is_some() { // change ui so it isnt stale on a disconnect.
                self.last_packet_time = None;
                if self.current_stage != LunabotStage::SoftStop {
                    self.current_stage = LunabotStage::SoftStop;
                    new_state = Some(LunabotStage::SoftStop);
                }
            }
        }

        if send_packet_rx_signal {
            self.base_mut().emit_signal("packet_received", &[]);
        }

        if let Some(stage) = new_state {
            self.base_mut()
                .emit_signal("stage_changed", &[Variant::from(stage as i32)]);
        }

        if self.first_connect
            && let Some(last_packet_time) = self.last_packet_time
            && last_packet_time.elapsed().as_millis() < 500
        {
            println!("sending first connect apriltag thing");
            self.send_toggle_apriltags(self.apriltags_enabled);
            self.first_connect = false;
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
        self.first_connect = true;
        let addr = match address_str.parse::<Ipv4Addr>() {
            Ok(addr) => {
                godot_warn!("Connecting to: {address_str}");
                addr
            }
            Err(_) => {
                godot_error!("Failed to parse address: {address_str}");
                return;
            }
        };

        // ensure to abort previous tasks so that there isnt anything lurking in the background
        if let Some(prev) = self.bg_task.take() {
            prev.abort();
        }

        let server_addr = SocketAddr::V4(SocketAddrV4::new(addr, common::ports::TELEOP));

        let (tx_outgoing, rx_outgoing) = tokio::sync::mpsc::unbounded_channel::<FromLunabase>();

        let ka_state = self.ka_state.clone();
        let outgoing_shared = self.outgoing.clone();
        let errored_tasks = self.errored_tasks.clone();
        let bt_status_msg = self.bt_status_msg.clone();
        let global_position = self.global_position.clone();
        let orientation = self.orientation.clone();

        let our_tx = tx_outgoing.clone();

        let task = get_tokio_handle().spawn(async move {
            let bind: SocketAddr = "0.0.0.0:0".parse().unwrap();
            let client = match QuicClient::connect_insecure(
                bind,
                server_addr,
                "lunabot",
                LunabotStage::SoftStop,
            )
            .await
            {
                Ok(c) => c,
                Err(e) => {
                    eprintln!("[LunabaseConnection] Connect failed: {e}");
                    return;
                }
            };

            // Open 4 multiplexed streams
            let command_stream = match client
                .open_bi::<FromLunabase, FromLunabot, { COMMAND_STREAM_ID }>()
                .await
            {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[LunabaseConnection] open command stream failed: {e}");
                    return;
                }
            };

            let pose_stream = match client
                .open_bi::<FromLunabase, FromLunabot, { POSE_STREAM_ID }>()
                .await
            {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[LunabaseConnection] open pose stream failed: {e}");
                    return;
                }
            };

            let error_stream = match client
                .open_bi::<FromLunabase, FromLunabot, { ERROR_STREAM_ID }>()
                .await
            {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[LunabaseConnection] open error stream failed: {e}");
                    return;
                }
            };

            let bt_status_stream = match client
                .open_bi::<FromLunabase, FromLunabot, { BT_STATUS_STREAM_ID }>()
                .await
            {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("[LunabaseConnection] open BT status stream failed: {e}");
                    return;
                }
            };

            let our_ka_state = client.keep_alive.shared_state();
            *ka_state.lock().unwrap() = Some(Arc::clone(&our_ka_state));

            // Recv pose data (RobotIsometry, ArmAngles)
            let global_position_c = global_position.clone();
            let orientation_c = orientation.clone();
            let pose_handle = tokio::spawn(async move {
                loop {
                    match pose_stream.recv().await {
                        Ok(msg) => match msg {
                            FromLunabot::RobotIsometry { origin, quat } => {
                                if let Ok(mut pos) = global_position_c.lock() {
                                    *pos = origin;
                                }
                                if let Ok(mut rot) = orientation_c.lock() {
                                    *rot = quat;
                                }
                            }
                            FromLunabot::ArmAngles { .. } => {}
                            _ => {}
                        },
                        Err(e) => {
                            eprintln!("[LunabaseConnection] Pose recv error: {e}");
                            break;
                        }
                    }
                }
            });

            // Recv error messages (ErroredTasks)
            let error_handle = tokio::spawn(async move {
                loop {
                    match error_stream.recv().await {
                        Ok(FromLunabot::ErroredTasks(map)) => {
                            if let Ok(mut guard) = errored_tasks.lock() {
                                *guard = map;
                            }
                        }
                        Ok(_) => {}
                        Err(e) => {
                            eprintln!("[LunabaseConnection] Error recv error: {e}");
                            break;
                        }
                    }
                }
            });

            // Recv Behavior tree status
            let bt_status_handle = tokio::spawn(async move {
                loop {
                    match bt_status_stream.recv().await {
                        Ok(FromLunabot::BTStatus(status_msg)) => {
                            if let Ok(mut guard) = bt_status_msg.lock() {
                                *guard = status_msg;
                            }
                        }
                        Ok(_) => {}
                        Err(e) => {
                            eprintln!("[LunabaseConnection] Error recv error: {e}");
                            break;
                        }
                    }
                }
            });

            // Send commands on command stream
            let mut rx = rx_outgoing;
            let cmd_handle = tokio::spawn(async move {
                while let Some(msg) = rx.recv().await {
                    if let Err(e) = command_stream.send(&msg).await {
                        eprintln!("[LunabaseConnection] Send error: {e}");
                        break;
                    }
                }
            });

            // Capture abort handles before select! so that whichever branch
            // wins, we can still kill the other inner stream tasks (dropping a
            // JoinHandle does not abort its task).
            let pose_abort = pose_handle.abort_handle();
            let error_abort = error_handle.abort_handle();
            let cmd_abort = cmd_handle.abort_handle();
            let bt_status_abort = bt_status_handle.abort_handle();

            // Watchdog thing: tear down the entire connection the moment vital
            // task or the QUIC connection itself dies. Otherwise keep-alive
            // datagrams could keep flowing while the streams are dead, leaving
            // both ends thinking they're still connected which used to happen every so often
            let conn_closed = client.quinn().closed();
            tokio::select! {
                _ = pose_handle => {
                    eprintln!("[LunabaseConnection] Pose task ended");
                }
                _ = error_handle => {
                    eprintln!("[LunabaseConnection] Error task ended");
                }
                _ = cmd_handle => {
                    eprintln!("[LunabaseConnection] Command task ended");
                }
                _ = bt_status_handle => {
                    eprintln!("[LunabaseConnection] BT Status task ended");
                }
                _ = conn_closed => {
                    eprintln!("[LunabaseConnection] QUIC connection closed");
                }
            }
            // closed now, release resources
            pose_abort.abort();
            error_abort.abort();
            cmd_abort.abort();
            bt_status_abort.abort();

            eprintln!("[LunabaseConnection] Tearing down connection");
            client.close();

            // A reconnect could have raced and
            // installed a newer ka_state / outgoing that we cant be clobbering
            {
                let mut g = ka_state.lock().unwrap();
                if g.as_ref().is_some_and(|a| Arc::ptr_eq(a, &our_ka_state)) {
                    *g = None;
                }
            }
            {
                let mut g = outgoing_shared.lock().unwrap();
                if g.as_ref().is_some_and(|s| s.same_channel(&our_tx)) {
                    *g = None;
                }
            }
        });

        *self.outgoing.lock().unwrap() = Some(tx_outgoing);
        self.bg_task = Some(task.abort_handle());
        godot_print!("Connection initiated to {address_str}");
    }

    #[func]
    fn reconnect(&mut self, address: GString) {
        let address_str = address.to_string();
        self.connect_to_address(address_str);
    }

    #[func]
    fn get_ms_since_last_packet(&self) -> i64 {
        let guard = self.ka_state.lock().unwrap();
        if let Some(ref state_arc) = *guard {
            state_arc
                .lock()
                .unwrap()
                .time_since_last()
                .map(|d| d.as_millis() as i64)
                .unwrap_or(99999)
        } else {
            99999
        }
    }

    #[func]
    fn get_lunabot_stage(&self) -> i32 {
        self.current_stage as i32
    }

    fn send_msg(&mut self, msg: FromLunabase) {
        let mut guard = self.outgoing.lock().unwrap();
        if let Some(tx) = guard.as_ref() {
            if let Err(e) = tx.send(msg) {
                godot_warn!(
                    "Failed to send message: {e}. Channel closed, clearing connection state."
                );
                *guard = None;
            }
        } else {
            godot_warn!("Cannot send: not connected");
        }
    }

    #[func]
    fn execute_steering(&mut self, left: f64, right: f64) {
        self.send_msg(FromLunabase::Steering(Steering::new(
            left,
            right,
            self.current_weight,
        )));
    }

    #[func]
    /// sigma spatial is in cm
    fn send_set_sigma_spatial(&mut self, new_sigma_spatial: f32) {
        self.send_msg(FromLunabase::SetSigmaSpatial(new_sigma_spatial / 100.0))
    }

    #[func]
    /// sigma range is in cm
    fn send_set_sigma_range(&mut self, new_sigma_range: f32) {
        self.send_msg(FromLunabase::SetSigmaRange(new_sigma_range / 100.0))
    }

    #[func]
    fn send_lift_actuators(&mut self, speed: f64) {
        self.send_msg(FromLunabase::set_lift_actuator(speed));
    }

    #[func]
    fn send_reset_obstacles(&mut self) {
        self.send_msg(FromLunabase::ResetObstacles);
    }

    #[func]
    fn send_toggle_apriltags(&mut self, on: bool) {
        self.apriltags_enabled = on;
        if on {
            self.send_msg(FromLunabase::EnableApriltags);
        } else {
            self.send_msg(FromLunabase::DisableApriltags);
        }
    }

    #[func]
    fn send_bucket_actuators(&mut self, speed: f64) {
        self.send_msg(FromLunabase::set_bucket_actuator(speed));
    }

    #[func]
    fn send_soft_stop(&mut self) {
        self.send_msg(FromLunabase::SoftStop);
    }

    /// continue mission means manual
    #[func]
    fn send_continue_mission(&mut self) {
        self.send_msg(FromLunabase::Manual);
    }

    #[func]
    fn send_start_autonomy(&mut self) {
        self.send_msg(FromLunabase::Navigate((0.0, 0.0)));
    }

    #[func]
    fn get_errored_tasks(&self) -> Dictionary<GString, GString> {
        let mut dict = Dictionary::new();
        if let Ok(guard) = self.errored_tasks.lock() {
            for (task_name, error_msg) in guard.iter() {
                dict.set(task_name.as_str(), error_msg.as_str());
            }
        }
        dict
    }

    #[func]
    fn get_bt_status(&self) -> String {
        if let Ok(status) = self.bt_status_msg.lock() {
            return status.deref().clone();
        } else {
            "".to_string()
        }
    }

    #[func]
    fn set_speed(&mut self, weight: f64) -> f64 {
        self.current_weight = weight;
        self.send_msg(FromLunabase::Steering(Steering::new(
            0.0,
            0.0,
            self.current_weight,
        )));
        self.current_weight
    }

    #[func]
    fn get_orientation(&mut self) -> PackedFloat32Array {
        let mut values_sent = PackedFloat32Array::new();
        if let Ok(orientation_values) = self.orientation.lock() {
            values_sent.clear();
            values_sent.push(orientation_values[0]);
            values_sent.push(orientation_values[1]);
            values_sent.push(orientation_values[2]);
            values_sent.push(orientation_values[3]);
        }
        values_sent
    }

    #[func]
    fn get_location(&mut self) -> PackedFloat32Array {
        let mut values_sent = PackedFloat32Array::new();
        if let Ok(position) = self.global_position.lock() {
            values_sent.clear();
            values_sent.push(position[0]);
            values_sent.push(position[1]);
            values_sent.push(position[2]);
        }
        values_sent
    }
}
