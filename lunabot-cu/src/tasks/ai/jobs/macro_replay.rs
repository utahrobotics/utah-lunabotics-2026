use bonsai_bt::Status::{Failure, Success};
use embedded_common::ActuatorCommand;
use tasker::tokio::{self, sync::mpsc};

use crate::tasks::ai::jobs::Job;

pub fn macro_replay_job(sequence: Option<Vec<(ActuatorCommand, f32)>>) -> Job<ActuatorCommand, ()> {
    let (output_tx, output_rx) = mpsc::channel(5);

    Job::spawn(
        async move {
            let Some(seq) = sequence else {
                return Failure;
            };
            for (cmd, delay) in seq {
                let _ = output_tx.send(cmd).await;
                if delay > 0.0 {
                    tokio::time::sleep(std::time::Duration::from_secs_f32(delay)).await;
                }
            }
            Success
        },
        output_rx,
        None,
    )
}

pub fn dig_macro_job() -> Job<ActuatorCommand, ()> {
    macro_replay_job(None)
}

pub fn load_macro_job() -> Job<ActuatorCommand, ()> {
    macro_replay_job(None)
}

pub fn dump_macro_job() -> Job<ActuatorCommand, ()> {
    macro_replay_job(None)
}
