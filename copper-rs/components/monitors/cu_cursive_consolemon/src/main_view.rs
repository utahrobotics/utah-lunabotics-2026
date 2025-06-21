use cursive::{
    event::Event,
    view::{View, CannotFocus, IntoBoxedView},
    views::{Panel, TextView},
    Printer, Vec2,
    event::EventResult,
    direction::Direction,
};
use cu29::config::CuConfig;
use std::sync::{Arc, Mutex};
use crate::{
    stats::{TaskStats, TaskStatus, PoolStats},
    dag::DagView,
    utils::get_system_info,
};

#[derive(PartialEq, Clone, Copy)]
pub enum Screen {
    Neofetch,
    Dag,
    Latency,
    MemoryPools,
}

// Main view that handles screen switching
pub struct MainView {
    pub active_screen: Screen,
    taskids: &'static [&'static str],
    pub task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
    pub task_stats: Arc<Mutex<TaskStats>>,
    pub pool_stats: Arc<Mutex<Vec<PoolStats>>>,
    config: CuConfig,
    content_view: Box<dyn View>,
}

impl MainView {
    pub fn new(
        taskids: &'static [&'static str],
        task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
        task_stats: Arc<Mutex<TaskStats>>,
        pool_stats: Arc<Mutex<Vec<PoolStats>>>,
        config: CuConfig,
    ) -> Self {
        let content_view = Panel::new(TextView::new(get_system_info()))
            .title("System Info")
            .into_boxed_view();

        Self {
            active_screen: Screen::Neofetch,
            taskids,
            task_statuses,
            task_stats,
            pool_stats,
            config,
            content_view,
        }
    }
    
    pub fn update_content(&mut self) {
        match self.active_screen {
            Screen::Neofetch => {
                self.content_view = Panel::new(TextView::new(get_system_info()))
                    .title("System Info")
                    .into_boxed_view();
            }
            Screen::Dag => {
                let dag_view = DagView::new(&self.config, self.task_statuses.clone());
                self.content_view = Panel::new(dag_view)
                    .title("Task DAG")
                    .into_boxed_view();
            }
            Screen::Latency => {
                let stats = self.task_stats.lock().unwrap();
                let mut content = String::new();
                content.push_str("Task Latency Statistics:\n\n");
                
                // Header
                content.push_str("┌─────────────────────────┬─────────────┬─────────────┬─────────────┐\n");
                content.push_str("│ Task Name               │ Min (μs)    │ Max (μs)    │ Avg (μs)    │\n");
                content.push_str("├─────────────────────────┼─────────────┼─────────────┼─────────────┤\n");
                
                // Task rows
                for (i, stat) in stats.stats.iter().enumerate() {
                    let task_name = if i < self.taskids.len() {
                        self.taskids[i]
                    } else {
                        "Unknown"
                    };
                    
                    let min_us = stat.min().as_nanos() as f64 / 1000.0;
                    let max_us = stat.max().as_nanos() as f64 / 1000.0;
                    let avg_us = stat.mean().as_nanos() as f64 / 1000.0;
                    
                    content.push_str(&format!(
                        "│ {:<23} │ {:>10}  │ {:>10}  │ {:>10}  │\n",
                        if task_name.len() > 23 { &task_name[..20] } else { task_name },
                        format!("{:.1}", min_us),
                        format!("{:.1}", max_us),
                        format!("{:.1}", avg_us)
                    ));
                }
                
                content.push_str("├─────────────────────────┼─────────────┼─────────────┼─────────────┤\n");
                
                // End-to-end row
                let e2e_min_us = stats.end2end.min().as_nanos() as f64 / 1000.0;
                let e2e_max_us = stats.end2end.max().as_nanos() as f64 / 1000.0;
                let e2e_avg_us = stats.end2end.mean().as_nanos() as f64 / 1000.0;
                
                content.push_str(&format!(
                    "│ {:<23} │ {:>10}  │ {:>10}  │ {:>10}  │\n",
                    "End-to-End",
                    format!("{:.1}", e2e_min_us),
                    format!("{:.1}", e2e_max_us),
                    format!("{:.1}", e2e_avg_us)
                ));
                
                content.push_str("└─────────────────────────┴─────────────┴─────────────┴─────────────┘\n\n");
                
                content.push_str("Units: μs = microseconds (1,000 μs = 1 ms)\n");
                content.push_str("Good latency: < 1,000 μs | Acceptable: < 10,000 μs | Poor: > 10,000 μs\n\n");
                content.push_str("Press 'r' to reset statistics");
                
                self.content_view = Panel::new(TextView::new(content))
                    .title("Latencies")
                    .into_boxed_view();
            }
            Screen::MemoryPools => {
                let pools = self.pool_stats.lock().unwrap();
                let mut content = String::new();
                content.push_str("Memory Pool Statistics:\n\n");
                
                // Header
                content.push_str("┌─────────────────────┬─────────────┬─────────────┬─────────────┬─────────────┐\n");
                content.push_str("│ Pool ID             │ Used/Total  │ Buffer Size │ Usage %     │ Rate/sec    │\n");
                content.push_str("├─────────────────────┼─────────────┼─────────────┼─────────────┼─────────────┤\n");
                
                for pool in pools.iter() {
                    let usage_percent = if pool.total_size > 0 {
                        (pool.handles_in_use as f64 / pool.total_size as f64) * 100.0
                    } else {
                        0.0
                    };
                    
                    let buffer_size_str = if pool.buffer_size >= 1024 * 1024 {
                        format!("{:.1}MB", pool.buffer_size as f64 / (1024.0 * 1024.0))
                    } else if pool.buffer_size >= 1024 {
                        format!("{:.1}KB", pool.buffer_size as f64 / 1024.0)
                    } else {
                        format!("{}B", pool.buffer_size)
                    };
                    
                    content.push_str(&format!(
                        "│ {:<19} │ {:>4}/{:<7}│ {:>10}  │ {:>8.1}%   │ {:>10}  │\n",
                        if pool.id.len() > 19 { &pool.id[..16] } else { &pool.id },
                        pool.handles_in_use,
                        pool.total_size,
                        buffer_size_str,
                        usage_percent,
                        pool.handles_per_second
                    ));
                }
                
                content.push_str("└─────────────────────┴─────────────┴─────────────┴─────────────┴─────────────┘\n\n");
                content.push_str("Usage colors: Green < 50% | Yellow 50-80% | Red > 80%");
                
                self.content_view = Panel::new(TextView::new(content))
                    .title("Memory Pools")
                    .into_boxed_view();
            }
        }
    }
    
    pub fn switch_screen(&mut self, screen: Screen) {
        if self.active_screen != screen {
            self.active_screen = screen;
            self.update_content();
        }
    }
}

impl View for MainView {
    fn draw(&self, printer: &Printer) {
        self.content_view.draw(printer);
    }
    
    fn on_event(&mut self, event: Event) -> EventResult {
        match event {
            Event::Char('1') => {
                self.switch_screen(Screen::Neofetch);
                EventResult::Consumed(None)
            }
            Event::Char('2') => {
                self.switch_screen(Screen::Dag);
                EventResult::Consumed(None)
            }
            Event::Char('3') => {
                self.switch_screen(Screen::Latency);
                EventResult::Consumed(None)
            }
            Event::Char('4') => {
                self.switch_screen(Screen::MemoryPools);
                EventResult::Consumed(None)
            }
            Event::Char('r') => {
                if self.active_screen == Screen::Latency {
                    self.task_stats.lock().unwrap().reset();
                    self.update_content(); // Refresh the content
                }
                EventResult::Consumed(None)
            }
            _ => {
                // Forward other events to the content panel
                self.content_view.on_event(event)
            }
        }
    }
    
    fn layout(&mut self, size: Vec2) {
        self.content_view.layout(size);
    }
    
    fn required_size(&mut self, constraint: Vec2) -> Vec2 {
        self.content_view.required_size(constraint)
    }
    
    fn take_focus(&mut self, source: Direction) -> Result<EventResult, CannotFocus> {
        self.content_view.take_focus(source)
    }
} 