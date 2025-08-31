use cu29::prelude::*;
use fxhash::{FxHashMap, FxHasher};
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// I got tired of dealing with TUI shenannigans so I made something much simpler.
pub struct SimpleMonitor {
    tasks: &'static [&'static str],
    errored_tasks: Arc<Mutex<FxHashMap<usize, (CuTaskState, String, Instant)>>>,
    last_print: Arc<Mutex<Instant>>,
}

impl CuMonitor for SimpleMonitor {
    fn new(_config: &CuConfig, taskids: &'static [&str]) -> CuResult<Self> {
        Ok(SimpleMonitor {
            tasks: taskids,
            errored_tasks: Arc::new(Mutex::new(FxHashMap::default())),
            last_print: Arc::new(Mutex::new(Instant::now())),
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let errored_tasks = Arc::clone(&self.errored_tasks);
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
                    let mut errors = errored_tasks.lock().unwrap();
                    let cutoff_time = now - Duration::from_millis(500);
                    errors.retain(|_, (_, _, timestamp)| *timestamp > cutoff_time);

                    if !errors.is_empty() {
                        println!("\n=== ERRORED TASKS ===");
                        for (&task_id, (state, error, _)) in errors.iter() {
                            let task_name =
                                tasks.get(task_id).map(|&name| name).unwrap_or("Unknown");
                            println!(
                                "Task {}: {} (State: {:?}) - {}",
                                task_id, task_name, state, error
                            );
                        }
                        println!("=====================\n");
                    } else {
                        println!("No errored tasks currently.");
                    }
                }
            }
        });

        Ok(())
    }

    fn process_copperlist(&self, _msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
        let mut errors = self.errored_tasks.lock().unwrap();
        errors.insert(taskid, (step, error.to_string(), Instant::now()));
        Decision::Ignore
    }

    fn stop(&mut self, clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
