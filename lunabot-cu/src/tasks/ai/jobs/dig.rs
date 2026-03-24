use std::thread::sleep;

use bonsai_bt::Status::{Running, Success};
use embedded_common::{Actuator, ActuatorCommand};
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

pub fn dig_job() -> Job<ActuatorCommand, ()> {
    // Lowkey have no idea what this does but it compiles
    let (output_tx, output_rx) = mpsc::channel(5);
    let (input_tx, input_rx) = mpsc::channel(5);

    println!("Yay digging time!");

    Job::spawn(
        async move {
            // Move the bucket down
            let _ = output_tx
                .send(ActuatorCommand::forward(Actuator::Bucket))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(1.0, Actuator::Bucket))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::backward(Actuator::Lift))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(1.0, Actuator::Lift))
                .await;
            tokio::time::sleep(std::time::Duration::from_secs_f32(0.5)).await;

            let _ = output_tx
                .send(ActuatorCommand::backward(Actuator::Bucket))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(0.0, Actuator::Lift))
                .await;
            tokio::time::sleep(std::time::Duration::from_secs_f32(0.5)).await;

            let _ = output_tx
                .send(ActuatorCommand::forward(Actuator::Lift))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(1.0, Actuator::Lift))
                .await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(0.0, Actuator::Bucket))
                .await;

            tokio::time::sleep(std::time::Duration::from_secs(1)).await;
            let _ = output_tx
                .send(ActuatorCommand::set_speed(0.0, Actuator::Lift))
                .await;

            Success
        },
        output_rx,
        input_tx,
    )
}
