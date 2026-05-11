use bonsai_bt::Status::Success;
use embedded_common::{Actuator, ActuatorCommand, Direction};
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

pub fn dump_job() -> Job<ActuatorCommand, ()> {
    let (output_tx, output_rx) = mpsc::channel(2);
    Job::spawn(
        async move {
            let macro_sequence: Option<Vec<(ActuatorCommand, f32)>> = None; // TODO: load macro sequence from file

            if let Some(sequence) = macro_sequence {
                for (cmd, delay) in sequence {
                    let _ = output_tx.send(cmd).await;
                    if delay > 0.0 {
                        tokio::time::sleep(std::time::Duration::from_secs_f32(delay)).await;
                    }
                }
                return Success;
            }

            // Fallback to old speed-based logic
            // send dump command
            let _ = output_tx
                .send(ActuatorCommand::set_speed(1.0, Actuator::Dumper, Direction::Forward))
                .await;
            tokio::time::sleep(std::time::Duration::from_secs_f32(1.5)).await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(1.0, Actuator::Dumper, Direction::Backward))
                .await;
            Success
        },
        output_rx,
        None,
    )
}
