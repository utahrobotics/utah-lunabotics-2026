use std::{collections::VecDeque, time::Duration};

use crate::ROBOT_STATE;
#[cfg(not(feature = "resim"))]
use crate::comms::LunabaseConnection;
use common::{FromLunabase, FromLunabot};
use cu29::{cutask::Freezable, prelude::CuBridge, rx_channels, tx_channels};
pub struct Lunabase {
    #[cfg(not(feature = "resim"))]
    connection: LunabaseConnection,
    #[cfg(not(feature = "resim"))]
    message_buffer: VecDeque<FromLunabase>,
    #[cfg(not(feature = "resim"))]
    max_pong_delay: Duration,
    #[cfg(not(feature = "resim"))]
    last_errored_tasks_packet: u64,
}

rx_channels! {
    pub struct FromLunabaseChannel : FromLunabaseTxId {
        // id, type, route
        from_lunabase_rx => FromLunabase = "lunabase_bridge/from_lunabase"
    }
}

tx_channels! {

    pub struct ToLunabaseChannel : ToLunabaseRxId {
        // id, type, route
        to_lunabase => FromLunabot = "lunabase_bridge/to_lunabase"
    }
}

impl Freezable for Lunabase {}

impl CuBridge for Lunabase {
    type Tx = ToLunabaseChannel;

    type Rx = FromLunabaseChannel;

    #[cfg(feature = "resim")]
    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _tx_channels: &[cu29::prelude::BridgeChannelConfig<
            <Self::Tx as cu29::prelude::BridgeChannelSet>::Id,
        >],
        _rx_channels: &[cu29::prelude::BridgeChannelConfig<
            <Self::Rx as cu29::prelude::BridgeChannelSet>::Id,
        >],
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    #[cfg(not(feature = "resim"))]
    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _tx_channels: &[cu29::prelude::BridgeChannelConfig<
            <Self::Tx as cu29::prelude::BridgeChannelSet>::Id,
        >],
        _rx_channels: &[cu29::prelude::BridgeChannelConfig<
            <Self::Rx as cu29::prelude::BridgeChannelSet>::Id,
        >],
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        use cu29::CuError;

        let max_pong_delay = if let Some(cfg) = config {
            Duration::from_millis(cfg.get("max_pong_delay_ms").unwrap_or(1000))
        } else {
            Duration::from_millis(1000)
        };

        let connection = LunabaseConnection::new()
            .map_err(|e| CuError::from(format!("Failed to create LunabaseConnection: {}", e)))?;

        Ok(Self {
            connection,
            message_buffer: VecDeque::new(),
            max_pong_delay,
            last_errored_tasks_packet: 0,
        })
    }

    #[cfg(feature = "resim")]
    fn send<'a, Payload>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        channel: &'static cu29::prelude::BridgeChannel<
            <Self::Tx as cu29::prelude::BridgeChannelSet>::Id,
            Payload,
        >,
        msg: &cu29::prelude::CuMsg<Payload>,
    ) -> cu29::CuResult<()>
    where
        Payload: cu29::prelude::CuMsgPayload + 'a,
    {
        Ok(())
    }

    #[cfg(not(feature = "resim"))]
    fn send<'a, Payload>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        _channel: &'static cu29::prelude::BridgeChannel<
            <Self::Tx as cu29::prelude::BridgeChannelSet>::Id,
            Payload,
        >,
        msg: &cu29::prelude::CuMsg<Payload>,
    ) -> cu29::CuResult<()>
    where
        Payload: cu29::prelude::CuMsgPayload + Clone + 'a + std::any::Any,
    {
        use crate::{simple_monitor::ERRORED_TASKS, utils::secs_to_nanos};
        use common::{FromLunabot, LUNABOT_STAGE};

        let heartbeat =
            bincode::encode_to_vec(LUNABOT_STAGE.load(), bincode::config::standard()).unwrap();
        self.connection.server.set_keep_alive_msg(&heartbeat);

        if let Some(errored_tasks) = ERRORED_TASKS.get()
            && clock.now().as_nanos() - self.last_errored_tasks_packet > secs_to_nanos(3.0)
        {
            if let Ok(errored_tasks) = errored_tasks.lock() {
                let transformed_tasks: std::collections::HashMap<String, String> = errored_tasks
                    .iter()
                    .map(|(_, (task_id, state, name, _instant))| {
                        (task_id.to_string(), format!("{:?}: {}", state, name))
                    })
                    .collect();
                let _ = self
                    .connection
                    .try_send(FromLunabot::ErroredTasks(transformed_tasks));
                self.last_errored_tasks_packet = clock.now().as_nanos();
            }
        }

        if let Some(payload) = msg.payload() {
            if let Some(lunabot_msg) = (payload as &dyn std::any::Any).downcast_ref::<FromLunabot>()
            {
                // you will probably want to rate limit how often the iso gets sent over for bandwidth reasons

                use core::f32;
                if let Err(e) = self.connection.try_send(lunabot_msg.clone()) {
                    eprintln!("Failed to send message to Lunabase: {}", e);
                }
                // send FromLunabot packets here :)
                let isometries = ROBOT_STATE
                    .get()
                    .unwrap()
                    .kinematic_root
                    .get_global_isometry();
                let position = isometries.translation.vector.cast::<f32>().data.0[0];

                let orientation = isometries.rotation.as_vector().cast::<f32>().data.0[0];

                if let Err(e) = self.connection.try_send(FromLunabot::RobotIsometry {
                    origin: (position),
                    quat: (orientation),
                }) {
                    eprintln!("cannot send isometries to lunabase: {}", e);
                }
                let velo = ROBOT_STATE.get().unwrap().get_velocity();

                let accel = ROBOT_STATE.get().unwrap().get_acceleration();

                let velof32 = [
                    velo[0] as f32, 
                    velo[1] as f32, 
                    velo[2] as f32
                ];
                let accelf32 = [
                    accel[0] as f32,
                    accel[1] as f32,
                    accel[2] as f32,
                ];
                if let Err(e) = self.connection.try_send(FromLunabot::RobotMotion { 
                    velocity: (velof32),
                    acceleration: (accelf32)
                 }){
                    eprintln!("cannot send motion to lunabase{}" , e);
                 }


            }
        }

        Ok(())
    }

    fn receive<'a, Payload>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        _channel: &'static cu29::prelude::BridgeChannel<
            <Self::Rx as cu29::prelude::BridgeChannelSet>::Id,
            Payload,
        >,
        msg: &mut cu29::prelude::CuMsg<Payload>,
    ) -> cu29::CuResult<()>
    where
        Payload: cu29::prelude::CuMsgPayload + 'a + std::any::Any,
    {
        #[cfg(not(feature = "resim"))]
        {
            use crate::{simple_monitor::ERRORED_TASKS, utils::secs_to_nanos};
            use common::{FromLunabot, LUNABOT_STAGE};
            use cu29::CuError;

            // Send heartbeat and robot state (moved from send method since send might not be called)
            let heartbeat =
                bincode::encode_to_vec(LUNABOT_STAGE.load(), bincode::config::standard()).unwrap();
            self.connection.server.set_keep_alive_msg(&heartbeat);

            // Send errored tasks periodically
            if let Some(errored_tasks) = ERRORED_TASKS.get()
                && clock.now().as_nanos() - self.last_errored_tasks_packet > secs_to_nanos(3.0)
            {
                if let Ok(errored_tasks) = errored_tasks.lock() {
                    let transformed_tasks: std::collections::HashMap<String, String> =
                        errored_tasks
                            .iter()
                            .map(|(_, (task_id, state, name, _instant))| {
                                (task_id.to_string(), format!("{:?}: {}", state, name))
                            })
                            .collect();
                    let _ = self
                        .connection
                        .try_send(FromLunabot::ErroredTasks(transformed_tasks));
                    self.last_errored_tasks_packet = clock.now().as_nanos();
                }
            }

            // Receive messages from lunabase
            while let Some(incoming_msg) = self.connection.try_recv() {
                self.message_buffer.push_back(incoming_msg);
            }

            if self.message_buffer.len() > 5 {
                eprintln!(
                    "[WARNING] {} msgs in the from lunabase msg buffer",
                    self.message_buffer.len()
                );
            }

            if !self.connection.is_alive(self.max_pong_delay) {
                self.message_buffer.clear();
                // Set disconnect message as payload
                if std::any::TypeId::of::<Payload>() == std::any::TypeId::of::<FromLunabase>() {
                    let disconnect_msg = unsafe {
                        std::mem::transmute_copy::<FromLunabase, Payload>(&FromLunabase::Disconnect)
                    };
                    msg.set_payload(disconnect_msg);
                    msg.metadata.process_time.start = clock.now().into();
                }
                return Err(CuError::new_with_cause(
                    "lunabase not connected",
                    std::io::Error::other("lunabase disconnected - timeout"),
                ));
            }
            // Set the next message from buffer as payload
            if let Some(next_msg) = self.message_buffer.pop_front() {
                if std::any::TypeId::of::<Payload>() == std::any::TypeId::of::<FromLunabase>() {
                    let payload_msg =
                        unsafe { std::mem::transmute_copy::<FromLunabase, Payload>(&next_msg) };
                    println!("setting payload msg: {:?}", next_msg);
                    msg.set_payload(payload_msg);
                    msg.metadata.process_time.start = clock.now().into();
                }
            } else {
                msg.clear_payload();
            }
        }
        Ok(())
    }
}
