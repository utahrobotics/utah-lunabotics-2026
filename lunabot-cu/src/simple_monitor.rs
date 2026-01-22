use cu29::prelude::*;
use std::collections::HashMap;
use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

/// Used in the lunabase source task to send over any errored tasks, the id is the first string in the tuple
pub static ERRORED_TASKS: OnceLock<
    Arc<Mutex<HashMap<usize, (String, CuTaskState, String, Instant)>>>,
> = OnceLock::new();

// I got tired of dealing with TUI shenannigans so I made something much simpler.
pub struct SimpleMonitor {
    tasks: &'static [&'static str],
    last_print: Arc<Mutex<Instant>>,
}

impl CuMonitor for SimpleMonitor {
    fn new(_config: &CuConfig, taskids: &'static [&str]) -> CuResult<Self> {
        ERRORED_TASKS.get_or_init(|| Arc::new(Mutex::new(HashMap::new())));
        Ok(SimpleMonitor {
            tasks: taskids,
            last_print: Arc::new(Mutex::new(Instant::now())),
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let last_print = Arc::clone(&self.last_print);
        let tasks = self.tasks;

        thread::spawn(move || {
            loop {
                thread::sleep(Duration::from_millis(100));

                let now = Instant::now();
                let should_print = {
                    let mut last = last_print.lock().unwrap();
                    if now.duration_since(*last) >= Duration::from_secs(3) {
                        *last = now;
                        true
                    } else {
                        false
                    }
                };

                if should_print {
                    let mut errors = ERRORED_TASKS.get().unwrap().lock().unwrap();
                    let cutoff_time = now - Duration::from_millis(500);
                    errors.retain(|_, (_, _, _, timestamp)| *timestamp > cutoff_time);

                    if !errors.is_empty() {
                        println!("\n=== ERRORED TASKS ===");
                        for (&task_id, (_, state, error, _)) in errors.iter() {
                            let task_name =
                                tasks.get(task_id).map(|&name| name).unwrap_or("Unknown");
                            println!(
                                "Task {}: {} (State: {:?}) - {}",
                                task_id, task_name, state, error
                            );
                        }
                        println!("=====================\n");
                    } else {
                        // println!("No errored tasks currently.");
                    }
                }
            }
        });

        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        for (task_id, msg) in msgs
            .iter()
            .filter(|msg| !msg.process_time.start.is_none() && !msg.process_time.end.is_none())
            .enumerate()
        {
            let process_duration =
                msg.process_time.end.unwrap().0 - msg.process_time.start.unwrap().0;
            if process_duration > 2_000_000 {
                warning!(
                    "Task {} process duration took suspiciously long: {} microseconds",
                    self.tasks[task_id],
                    (msg.process_time.end.unwrap().0 - msg.process_time.start.unwrap().0) / 1000
                );
            }
        }
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
        let mut errors = ERRORED_TASKS.get().unwrap().lock().unwrap();
        errors.insert(
            taskid,
            (
                self.tasks[taskid].to_string(),
                step,
                error.to_string(),
                Instant::now(),
            ),
        );
        Decision::Ignore
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
