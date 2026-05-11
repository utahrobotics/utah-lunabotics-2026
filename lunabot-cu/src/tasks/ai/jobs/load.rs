use bonsai_bt::Status::Success;
use embedded_common::{Actuator, ActuatorCommand, Direction};
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

pub fn load_job() -> Job<ActuatorCommand, ()> {
    // Async channels to communicate with motors
    let (output_tx, output_rx) = mpsc::channel(5);

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
            // Move the bucket to dump position
            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    1.0,
                    Actuator::Bucket,
                    Direction::Forward, // Dump the dirt in the bucket
                ))
                .await;

            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    1.0,
                    Actuator::Lift,
                    Direction::Backward,
                ))
                .await;

            // Wait for it to dump
            tokio::time::sleep(std::time::Duration::from_secs(1)).await;

            // Move back up
            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    1.0,
                    Actuator::Bucket,
                    Direction::Backward,
                ))
                .await;

            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    1.0,
                    Actuator::Lift,
                    Direction::Forward,
                ))
                .await;

            tokio::time::sleep(std::time::Duration::from_secs_f32(0.75)).await;

            // Stop the bucket
            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    0.0,
                    Actuator::Bucket,
                    Direction::Backward,
                ))
                .await;

            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    0.0,
                    Actuator::Lift,
                    Direction::Forward,
                ))
                .await;

            Success
        },
        output_rx,
        None,
    )
}
