use cursive::{
    event::Key, theme::{BorderStyle, Palette}, view::{Nameable, Resizable}, views::{LinearLayout, Panel, TextView}, Cursive, CursiveExt, With
};
use compact_str::CompactString;
use cu29::{
    clock::{CuDuration, RobotClock},
    config::CuConfig,
    cutask::CuMsgMetadata,
    monitoring::{CuMonitor, CuTaskState, Decision},
    prelude::{pool, CuCompactString},
    CuError, CuResult,
};
use std::{
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc, Mutex,
    },
    time::Duration,
    thread,
};
use crate::{
    stats::{TaskStats, TaskStatus, PoolStats},
    main_view::{MainView, Screen},
};

// The main monitor struct
pub struct CuCursiveConsoleMon {
    config: CuConfig,
    taskids: &'static [&'static str],
    task_stats: Arc<Mutex<TaskStats>>,
    task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
    quitting: Arc<AtomicBool>,
    pool_stats: Arc<Mutex<Vec<PoolStats>>>,
}

impl CuMonitor for CuCursiveConsoleMon {
    fn new(config: &CuConfig, taskids: &'static [&'static str]) -> CuResult<Self>
    where
        Self: Sized,
    {
        let task_stats = Arc::new(Mutex::new(TaskStats::new(
            taskids.len(),
            CuDuration::from(Duration::from_secs(5)),
        )));

        Ok(Self {
            config: config.clone(),
            taskids,
            task_stats,
            task_statuses: Arc::new(Mutex::new(vec![TaskStatus::default(); taskids.len()])),
            quitting: Arc::new(AtomicBool::new(false)),
            pool_stats: Arc::new(Mutex::new(Vec::new())),
        })
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        let config = self.config.clone();
        let taskids = self.taskids;
        let task_stats_ui = self.task_stats.clone();
        let task_statuses_ui = self.task_statuses.clone();
        let pool_stats_ui = self.pool_stats.clone();
        let quitting = self.quitting.clone();

        // Start the cursive UI in a separate thread
        thread::spawn(move || {
            let mut siv = Cursive::default();
            siv.set_theme(cursive::theme::Theme {
                shadow: true,
                borders: BorderStyle::Simple,
                palette: Palette::retro().with(|palette| {
                    use cursive::theme::BaseColor::*;
        
                    {
                        // First, override some colors from the base palette.
                        use cursive::theme::Color::TerminalDefault;
                        use cursive::style::PaletteColor::*;
        
                        palette[Background] = TerminalDefault;
                        palette[View] = TerminalDefault;
                        palette[Primary] = White.dark();
                        palette[TitlePrimary] = Blue.light();
                        palette[Secondary] = Blue.light();
                        palette[Highlight] = Blue.dark();
                    }
        
                    {
                        // Then override some styles.
                        use cursive::style::Effect::*;
                        use cursive::style::PaletteStyle::*;
                        use cursive::style::Style;
                        palette[Highlight] = Style::from(Blue.light()).combine(Bold);
                        palette[EditableTextCursor] = Style::secondary().combine(Reverse).combine(Underline)
                    }
                }),
            });
            // Create the main view
            let main_view = MainView::new(
                taskids,
                task_statuses_ui,
                task_stats_ui,
                pool_stats_ui,
                config,
            );

            // Build the UI with menu and content
            let menu_text = "[1] SysInfo  [2] DAG  [3] Latencies  [4] Memory Pools  [q] Quit";
            
            siv.add_layer(
                LinearLayout::vertical()
                    .child(
                        Panel::new(TextView::new(menu_text))
                            .title("Menu")
                            .fixed_height(3) // Fixed small height for menu
                    )
                    .child(main_view.with_name("main_view").full_screen()) // Main content takes remaining space
            );

            // Set up periodic refresh for real-time updates
            let cb_sink = siv.cb_sink().clone();
            let quitting_refresh = quitting.clone();
            thread::spawn(move || {
                loop {
                    if quitting_refresh.load(Ordering::SeqCst) {
                        break;
                    }
                    thread::sleep(Duration::from_millis(100)); // Update every 100ms
                    let _ = cb_sink.send(Box::new(|s: &mut Cursive| {
                        s.call_on_name("main_view", |view: &mut MainView| {
                            view.update_content();
                        });
                    }));
                }
            });

            // Global callbacks for screen switching
            siv.add_global_callback('1', |s| {
                s.call_on_name("main_view", |view: &mut MainView| {
                    view.switch_screen(Screen::Neofetch);
                });
            });
            siv.add_global_callback('2', |s| {
                s.call_on_name("main_view", |view: &mut MainView| {
                    view.switch_screen(Screen::Dag);
                });
            });
            siv.add_global_callback('3', |s| {
                s.call_on_name("main_view", |view: &mut MainView| {
                    view.switch_screen(Screen::Latency);
                });
            });
            siv.add_global_callback('4', |s| {
                s.call_on_name("main_view", |view: &mut MainView| {
                    view.switch_screen(Screen::MemoryPools);
                });
            });
            siv.add_global_callback('r', |s| {
                s.call_on_name("main_view", |view: &mut MainView| {
                    if view.active_screen == Screen::Latency {
                        view.task_stats.lock().unwrap().reset();
                        view.update_content();
                    }
                });
            });

            // Global callbacks
            siv.add_global_callback(Key::Esc, |s| s.quit());
            siv.add_global_callback('q', |s| s.quit());

            // Set up periodic refresh
            siv.set_fps(5); // Lower FPS since we have dedicated refresh timer

            siv.run();
            quitting.store(true, Ordering::SeqCst);
        });

        Ok(())
    }

    fn process_copperlist(&self, msgs: &[&CuMsgMetadata]) -> CuResult<()> {
        // Update task stats and statuses
        {
            let mut task_stats = self.task_stats.lock().unwrap();
            task_stats.update(msgs);
        }
        {
            let mut task_statuses = self.task_statuses.lock().unwrap();
            for (i, msg) in msgs.iter().enumerate() {
                if i < task_statuses.len() {
                    let CuCompactString(status_txt) = &msg.status_txt;
                    task_statuses[i].status_txt = CompactString::from(status_txt.as_str());
                    if !task_statuses[i].status_txt.is_empty()
                        && task_statuses[i].status_txt.as_bytes()[0] == 0
                    {
                        task_statuses[i].status_txt = CompactString::new("");
                    }
                }
            }
        }

        // Update pool statistics
        {
            let pool_stats_data = pool::pools_statistics();
            let mut pool_stats = self.pool_stats.lock().unwrap();
            for (id, space_left, total_size, buffer_size) in pool_stats_data {
                let id_str = id.to_string();
                if let Some(existing) = pool_stats.iter_mut().find(|p| p.id == id_str) {
                    existing.update(space_left, total_size);
                } else {
                    pool_stats.push(PoolStats::new(id_str, space_left, total_size, buffer_size));
                }
            }
        }

        if self.quitting.load(Ordering::SeqCst) {
            return Err("Exiting...".into());
        }
        Ok(())
    }

    fn process_error(&self, taskid: usize, step: CuTaskState, error: &CuError) -> Decision {
        {
            let mut task_statuses = self.task_statuses.lock().unwrap();
            if taskid < task_statuses.len() {
                let status = &mut task_statuses[taskid];
                if taskid == 0 {
                    println!("error: {}", error);
                }
                status.is_error = true;
                status.error = CompactString::from(error.to_string().as_str());
            }
        }
        match step {
            CuTaskState::Start => Decision::Shutdown,
            CuTaskState::Preprocess => Decision::Abort,
            CuTaskState::Process => Decision::Ignore,
            CuTaskState::Postprocess => Decision::Ignore,
            CuTaskState::Stop => Decision::Shutdown,
        }
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.task_stats
            .lock()
            .unwrap()
            .stats
            .iter_mut()
            .for_each(|s| s.reset());
        Ok(())
    }
} 