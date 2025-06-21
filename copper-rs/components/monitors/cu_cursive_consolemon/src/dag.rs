use cursive::{
    event::{Event, Key},
    view::{View, CannotFocus},
    Printer, Vec2,
    event::EventResult,
    direction::Direction,
};
use cu29::{
    config::{CuConfig, Node},
};
use std::{
    fmt::{Display, Formatter},
    sync::{Arc, Mutex},
};
use crate::stats::TaskStatus;

// Node types for the DAG
#[derive(Copy, Clone)]
pub enum NodeType {
    Unknown,
    Source,
    Sink,
    Task,
}

impl Display for NodeType {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Unknown => write!(f, "?"),
            Self::Source => write!(f, "⏩"),
            Self::Task => write!(f, "⚡"),
            Self::Sink => write!(f, "🏁"),
        }
    }
}

impl NodeType {
    fn add_incoming(self) -> NodeType {
        match self {
            Self::Unknown => Self::Sink,
            Self::Source => Self::Task,
            Self::Sink => Self::Sink,
            Self::Task => Self::Task,
        }
    }

    fn add_outgoing(self) -> NodeType {
        match self {
            Self::Unknown => Self::Source,
            Self::Source => Self::Source,
            Self::Sink => Self::Task,
            Self::Task => Self::Task,
        }
    }
}

// DAG view implementation
pub struct DagView {
    nodes: Vec<Node>,
    node_types: Vec<NodeType>,
    task_statuses: Arc<Mutex<Vec<TaskStatus>>>,
    scroll_x: usize,
    scroll_y: usize,
}

impl DagView {
    pub fn new(config: &CuConfig, task_statuses: Arc<Mutex<Vec<TaskStatus>>>) -> Self {
        let mut nodes = Vec::new();
        let mut node_types = Vec::new();

        if let Ok(graph) = config.get_graph(None) {
            for (_, node) in graph.get_all_nodes() {
                nodes.push(node.clone());
                node_types.push(NodeType::Unknown);
            }

            // Determine node types based on connections
            for (dst_index, _) in nodes.iter().enumerate() {
                if let Ok(incoming_edges) = graph.get_dst_edges(dst_index as u32) {
                    for _ in incoming_edges.iter() {
                        if let Some((src_index, dst_index)) = graph.0.edge_endpoints(
                            (*incoming_edges.first().unwrap_or(&0) as u32).into()
                        ) {
                            let (src_index, dst_index) = (src_index.index(), dst_index.index());
                            if dst_index < node_types.len() {
                                node_types[dst_index] = node_types[dst_index].add_incoming();
                            }
                            if src_index < node_types.len() {
                                node_types[src_index] = node_types[src_index].add_outgoing();
                            }
                        }
                    }
                }
            }
        }

        Self {
            nodes,
            node_types,
            task_statuses,
            scroll_x: 0,
            scroll_y: 0,
        }
    }
}

impl View for DagView {
    fn draw(&self, printer: &Printer) {
        let statuses = self.task_statuses.lock().unwrap();
        
        // Draw nodes in a grid layout
        let node_width = 25;
        let node_height = 4;
        let cols = printer.size.x as usize / node_width;
        
        for (idx, node) in self.nodes.iter().enumerate() {
            if idx >= statuses.len() {
                break;
            }
            
            let row = idx / cols.max(1);
            let col = idx % cols.max(1);
            
            let x = col * node_width;
            let y = row * node_height;
            
            // Skip if outside visible area due to scrolling
            if x < self.scroll_x || y < self.scroll_y {
                continue;
            }
            if x - self.scroll_x >= printer.size.x as usize || 
               y - self.scroll_y >= printer.size.y as usize {
                continue;
            }
            
            let screen_x = x - self.scroll_x;
            let screen_y = y - self.scroll_y;
            
            // Draw node border
            for i in 0..node_width.min(printer.size.x as usize - screen_x) {
                if screen_y < printer.size.y as usize {
                    printer.print((screen_x + i, screen_y), "─");
                }
                if screen_y + node_height - 1 < printer.size.y as usize {
                    printer.print((screen_x + i, screen_y + node_height - 1), "─");
                }
            }
            
            for i in 1..node_height - 1 {
                if screen_y + i < printer.size.y as usize {
                    if screen_x < printer.size.x as usize {
                        printer.print((screen_x, screen_y + i), "│");
                    }
                    if screen_x + node_width - 1 < printer.size.x as usize {
                        printer.print((screen_x + node_width - 1, screen_y + i), "│");
                    }
                }
            }
            
            // Draw corners
            if screen_x < printer.size.x as usize && screen_y < printer.size.y as usize {
                printer.print((screen_x, screen_y), "┌");
            }
            if screen_x + node_width - 1 < printer.size.x as usize && screen_y < printer.size.y as usize {
                printer.print((screen_x + node_width - 1, screen_y), "┐");
            }
            if screen_x < printer.size.x as usize && screen_y + node_height - 1 < printer.size.y as usize {
                printer.print((screen_x, screen_y + node_height - 1), "└");
            }
            if screen_x + node_width - 1 < printer.size.x as usize && screen_y + node_height - 1 < printer.size.y as usize {
                printer.print((screen_x + node_width - 1, screen_y + node_height - 1), "┘");
            }
            
            // Draw node content
            let status = &statuses[idx];
            let node_type = if idx < self.node_types.len() { 
                self.node_types[idx] 
            } else { 
                NodeType::Unknown 
            };
            
            if screen_y + 1 < printer.size.y as usize {
                let title = format!(" {} {}", node_type, node.get_id());
                let title = if title.len() > node_width - 2 {
                    format!("{}...", &title[..node_width - 5])
                } else {
                    title
                };
                printer.print((screen_x + 1, screen_y + 1), &title);
            }
            
            if screen_y + 2 < printer.size.y as usize {
                let status_line = if status.is_error {
                    format!(" ❌ ERROR: {}", 
                        if status.error.is_empty() { "Unknown error" } else { &status.error }
                    )
                } else {
                    format!(" ✅ OK: {}", status.status_txt)
                };
                let status_line = if status_line.len() > node_width - 2 {
                    format!("{}...", &status_line[..node_width - 5])
                } else {
                    status_line
                };
                printer.print((screen_x + 1, screen_y + 2), &status_line);
            }
        }

        drop(statuses);
        let mut statuses = self.task_statuses.lock().unwrap();
        for status in statuses.iter_mut() {
            if status.is_error {
                status.is_error = false;
            }
        }
    }
    
    fn on_event(&mut self, event: Event) -> EventResult {
        match event {
            Event::Key(Key::Left) | Event::Char('h') => {
                self.scroll_x = self.scroll_x.saturating_sub(5);
                EventResult::Consumed(None)
            }
            Event::Key(Key::Right) | Event::Char('l') => {
                self.scroll_x += 5;
                EventResult::Consumed(None)
            }
            Event::Key(Key::Up) | Event::Char('k') => {
                self.scroll_y = self.scroll_y.saturating_sub(1);
                EventResult::Consumed(None)
            }
            Event::Key(Key::Down) | Event::Char('j') => {
                self.scroll_y += 1;
                EventResult::Consumed(None)
            }
            _ => EventResult::Ignored,
        }
    }
    
    fn required_size(&mut self, _constraint: Vec2) -> Vec2 {
        Vec2::new(80, 24) // Default size
    }
    
    fn take_focus(&mut self, _source: Direction) -> Result<EventResult, CannotFocus> {
        Ok(EventResult::Consumed(None))
    }
} 