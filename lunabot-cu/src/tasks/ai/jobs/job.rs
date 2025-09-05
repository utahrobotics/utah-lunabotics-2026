use bonsai_bt::Status;
use cu29::prelude::*;
use tasker::tokio::sync::{mpsc, watch};
use tasker::tokio::task::JoinHandle;

/// Long running Action for bonsai
pub struct Job<Output> {
    status_rx: watch::Receiver<Status>,
    body_thread: JoinHandle<Status>,
    output: mpsc::Receiver<Output>,
}

impl<Output> Job<Output> {
    /// Returns new job in running state.
    /// The body should use the tx side of status_rx to send status updates
    /// The body should use the tx side of output_rx to enqueue output messages (i.e. Steering or ActuatorCommand)
    fn spawn<F>(
        body: F,
        status_rx: watch::Receiver<Status>,
        output_rx: mpsc::Receiver<Output>,
    ) -> Self
    where
        F: Future<Output = Status> + Send + 'static,
    {
        Self {
            body_thread: tasker::get_tokio_handle().spawn(body),
            status_rx,
            output: output_rx,
        }
    }

    fn get_status(&self) -> Status {
        *self.status_rx.borrow()
    }

    fn is_finished(&self) -> bool {
        self.body_thread.is_finished()
    }

    /// beware backpressure
    fn get_output(&mut self) -> Option<Output> {
        // ^ this function exists because I dont want people to call recv() manually in the coppertask and block the pipeline.
        if self.output.len() > 3 {
            warning!("More than 3 messgaes in the output queue for job {}");
        }
        self.output.try_recv().ok()
    }
}
