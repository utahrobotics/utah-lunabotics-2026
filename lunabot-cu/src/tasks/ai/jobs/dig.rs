use bonsai_bt::Status::{Success};
use embedded_common::{Actuator, ActuatorCommand, Direction};
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

pub fn dig_job() -> Job<ActuatorCommand, ()> {
    // Async channels to communicate with motors
    let (output_tx, output_rx) = mpsc::channel(5);

    Job::spawn(
        async move {
            // Move the bucket down
            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    1.0,
                    Actuator::Bucket,
                    Direction::Forward,
                ))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    1.0,
                    Actuator::Lift,
                    Direction::Backward,
                ))
                .await;
            tokio::time::sleep(std::time::Duration::from_secs_f32(0.5)).await;

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
            tokio::time::sleep(std::time::Duration::from_secs_f32(0.5)).await;

            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    1.0,
                    Actuator::Lift,
                    Direction::Forward,
                ))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    0.0,
                    Actuator::Bucket,
                    Direction::Backward,
                ))
                .await;

            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
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
