use bonsai_bt::Status::Success;
use embedded_common::{Actuator, ActuatorCommand, Direction};
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

pub fn dump_job() -> Job<ActuatorCommand, ()> {
    let (output_tx, output_rx) = mpsc::channel(2);
    Job::spawn(
        async move {
            // send dump command
            output_tx
                .send(ActuatorCommand::new(Actuator::Dumper, Direction::Positive))
                .await
                .unwrap();
            tokio::time::sleep(std::time::Duration::from_secs_f32(1.5)).await;
            output_tx
                .send(ActuatorCommand::new(Actuator::Dumper, Direction::Negative))
                .await
                .unwrap();
            Success
        },
        output_rx,
        None,
    )
}
