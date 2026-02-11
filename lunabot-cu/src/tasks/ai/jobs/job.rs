use bonsai_bt::Status::{self, Running};
use cu29::prelude::*;
use tasker::get_tokio_handle;
use tasker::tokio::sync::mpsc;
use tasker::tokio::task::JoinHandle;

/// Long running Action for bonsai
#[derive(Debug)]
pub struct Job<Output> {
    body_thread: JoinHandle<Status>,
    output: mpsc::Receiver<Output>,
}

impl<Output> Job<Output> {
    /// Returns new job in running state.
    /// The body should use the tx side of status_rx to send status updates
    /// The body should use the tx side of output_rx to enqueue output messages (i.e. Steering or ActuatorCommand)
    pub fn spawn<F>(
        body: F,
        output_rx: mpsc::Receiver<Output>,
    ) -> Self
    where
        F: Future<Output = Status> + Send + 'static,
    {
        Self {
            body_thread: tasker::get_tokio_handle().spawn(body),
            output: output_rx,
        }
    }

    pub fn cancel(&mut self) {
        println!("cancelling job");
        self.body_thread.abort();
    }

    /// if a job is finished, this will return the final status returned by the job body
    /// otherwise it will return the latest status sent by the job body.
    /// if the body returned running but the async body is finished, we ignore and report failure
    pub fn get_status(&mut self) -> Status {
        if self.is_finished() {
            let status = get_tokio_handle().block_on(async { (&mut self.body_thread).await });
            let status = status.unwrap_or(Status::Failure);
            if status == Running {
                Status::Failure
            } else {
                status
            }
        } else {
            Status::Running
        }
    }

    pub fn is_finished(&self) -> bool {
        self.body_thread.is_finished()
    }

    /// beware backpressure
    /// this function is non blocking
    pub fn get_output(&mut self) -> Option<Output> {
        // ^ this function exists because I dont want people to call recv() manually in the coppertask and block the pipeline.
        if self.output.len() > 3 {
            warning!("More than 3 messgaes in the output queue for job {}");
        }
        self.output.try_recv().ok()
    }
}
