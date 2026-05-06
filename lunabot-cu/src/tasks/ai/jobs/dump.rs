use bonsai_bt::Status::Success;
use embedded_common::{Actuator, ActuatorCommand, Direction};
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

const SEQUENCE: [(f32, f64, Direction, f64, Direction); 2] = [
    (1.0,  1.0, Direction::Forward,  1.0, Direction::Backward),
    (0.75, 1.0, Direction::Backward, 1.0, Direction::Forward),
];

pub fn dump_job() -> Job<ActuatorCommand, ()> {
    // Async channels to communicate with motors
    let (output_tx, output_rx) = mpsc::channel(5);

    Job::spawn(
        async move {
            for (time, bucket_speed, bucket_direction, lift_speed, lift_direction) in SEQUENCE {
                let _ = output_tx
                    .send(ActuatorCommand::set_speed(
                        bucket_speed,
                        Actuator::Bucket,
                        bucket_direction,
                    ))
                    .await;
                let _ = output_tx
                    .send(ActuatorCommand::set_speed(
                        lift_speed,
                        Actuator::Lift,
                        lift_direction,
                    ))
                    .await;
                tokio::time::sleep(std::time::Duration::from_secs_f32(time)).await;
            }
            
            // Zero speeds at end
            let _ = output_tx
                .send(ActuatorCommand::set_speed(
                    0.0,
                    Actuator::Bucket,
                    Direction::Forward,
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
