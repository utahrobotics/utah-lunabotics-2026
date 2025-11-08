use cu29::{
    CuError, CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuMsg, CuSrcTask, Freezable},
    output_msg,
};
use std::{collections::VecDeque, time::Duration};

#[cfg(not(feature = "resim"))]
use crate::comms::LunabaseConnection;
use common::FromLunabase;

pub struct Lunabase {
    #[cfg(not(feature = "resim"))]
    connection: LunabaseConnection,
    message_buffer: VecDeque<FromLunabase>,
    max_pong_delay: Duration,
    last_errored_tasks_packet: u64,
}

impl Freezable for Lunabase {}

impl CuSrcTask for Lunabase {
    type Output<'m> = output_msg!(Option<FromLunabase>);

    #[cfg(feature = "resim")]
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {
            message_buffer: VecDeque::new(),
            max_pong_delay: Duration::from_millis(1000),
            last_errored_tasks_packet: 0,
        })
    }

    #[cfg(not(feature = "resim"))]
    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
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

    fn process(&mut self, clock: &RobotClock, output: &mut Self::Output<'_>) -> CuResult<()> {
        #[cfg(not(feature = "resim"))]
        {
            use common::LUNABOT_STAGE;

            use crate::{
                simple_monitor::ERRORED_TASKS,
                utils::{nanos_to_secs, secs_to_nanos},
            };

            while let Some(msg) = self.connection.try_recv() {
                println!("msg: {:?}", msg);
                self.message_buffer.push_back(msg);
            }
            if self.message_buffer.len() > 5 {
                eprintln!(
                    "[WARNING] {} msgs in the from lunabase msg buffer",
                    self.message_buffer.len()
                );
            }

            if !self.connection.is_alive(self.max_pong_delay) {
                self.message_buffer.clear();
                // self.message_buffer.push_back(FromLunabase::Disconnect);
                output.set_payload(Some(FromLunabase::Disconnect));

                return Err(CuError::new_with_cause(
                    "lunabase not connected",
                    std::io::Error::other("lunabase disconnected - timeout"),
                ));
            }
            // should be a safe unwrap.
            let heartbeat =
                bincode::encode_to_vec(LUNABOT_STAGE.load(), bincode::config::standard()).unwrap();
            self.connection.server.set_keep_alive_msg(&heartbeat);

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
                    self.connection
                        .try_send(common::FromLunabot::ErroredTasks(transformed_tasks))
                        .map_err(|e| CuError::new_with_cause("lunabase not connected", e))?;
                    self.last_errored_tasks_packet = clock.now().as_nanos();
                }
            }
        }

        output.set_payload(self.message_buffer.pop_front());
        output.metadata.process_time.start = clock.now().into();

        Ok(())
    }
}
