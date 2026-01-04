use std::time::Duration;

use common::Steering;
use nalgebra::Vector2;
use simple_motion::StaticNode;
use tasker::tokio::{
    self,
    sync::{mpsc, watch},
};

use crate::tasks::ai::jobs::Job;

/// follows path fails if the robot fails to move significantly in stuck_timeout_secs
/// also could fail if this job is cancelled
pub fn follow_path_job(
    _stuck_timeout_secs: f32,
    chain: StaticNode,
    _path: Vec<Vector2<f32>>,
) -> Job<Steering> {
    let (status_tx, status_rx) = watch::channel(bonsai_bt::Status::Running);
    let (output_tx, output_rx) = mpsc::channel(5);
    Job::spawn(
        async move {
            let _robot_isometry = chain.get_global_isometry();
            loop {
                println!("sending steering value");
                // just sleep and then fail as an example but this loop should be following the path
                let _ = output_tx.send(Steering::new(1.0, 1.0, 5000.0)).await;
                // if you dont give a command to the vesc for more then ~1 second, as a saftey feature it will stop moving.
                // in order to get the robot to continuously move you have to continuously send commands.
                // For that reason, in this case it might be a good idea to keep track of the last known steering command from this job in the
                // match arm for this Action, and just always use the last known steering instead of "consuming" the steering commands requiring this
                // job to send more all the time.
                tokio::time::sleep(Duration::from_secs(1)).await;
                let _ = status_tx.send(bonsai_bt::Status::Failure);
                break;
            }
            let _ = output_tx.send(Steering::new(0.0, 0.0, 5000.0)).await;
            bonsai_bt::Status::Failure
        },
        status_rx,
        output_rx,
    )
}
