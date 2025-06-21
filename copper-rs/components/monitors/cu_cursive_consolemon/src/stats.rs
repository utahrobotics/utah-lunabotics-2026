use compact_str::{CompactString, ToCompactString};
use cu29::{
    clock::CuDuration,
    cutask::CuMsgMetadata,
    monitoring::CuDurationStatistics,
};
use std::time::Instant;

pub struct TaskStats {
    pub stats: Vec<CuDurationStatistics>,
    pub end2end: CuDurationStatistics,
}

impl TaskStats {
    pub fn new(num_tasks: usize, max_duration: CuDuration) -> Self {
        let stats = vec![CuDurationStatistics::new(max_duration); num_tasks];
        TaskStats {
            stats,
            end2end: CuDurationStatistics::new(max_duration),
        }
    }

    pub fn update(&mut self, msgs: &[&CuMsgMetadata]) {
        for (i, &msg) in msgs.iter().enumerate() {
            let (before, after) = (
                msg.process_time.start.unwrap(),
                msg.process_time.end.unwrap(),
            );
            self.stats[i].record(after - before);
        }
        self.end2end.record(compute_end_to_end_latency(msgs));
    }

    pub fn reset(&mut self) {
        for s in &mut self.stats {
            s.reset();
        }
        self.end2end.reset();
    }
}

fn compute_end_to_end_latency(msgs: &[&CuMsgMetadata]) -> CuDuration {
    msgs.last().unwrap().process_time.end.unwrap()
        - msgs.first().unwrap().process_time.start.unwrap()
}

#[derive(Default, Clone)]
pub struct TaskStatus {
    pub is_error: bool,
    pub status_txt: CompactString,
    pub error: CompactString,
}

pub struct PoolStats {
    pub id: CompactString,
    pub space_left: usize,
    pub total_size: usize,
    pub buffer_size: usize,
    pub handles_in_use: usize,
    pub handles_per_second: usize,
    pub last_update: Instant,
    pub prev_handles_in_use: usize,
}

impl PoolStats {
    pub fn new(
        id: impl ToCompactString,
        space_left: usize,
        total_size: usize,
        buffer_size: usize,
    ) -> Self {
        Self {
            id: id.to_compact_string(),
            space_left,
            total_size,
            buffer_size,
            handles_in_use: total_size - space_left,
            handles_per_second: 0,
            last_update: Instant::now(),
            prev_handles_in_use: 0,
        }
    }

    pub fn update(&mut self, space_left: usize, total_size: usize) {
        let now = Instant::now();
        let handles_in_use = total_size - space_left;
        let elapsed = now.duration_since(self.last_update).as_secs_f32();

        if elapsed >= 1.0 {
            self.handles_per_second =
                ((handles_in_use.abs_diff(self.handles_in_use)) as f32 / elapsed) as usize;
            self.prev_handles_in_use = self.handles_in_use;
            self.last_update = now;
        }

        self.handles_in_use = handles_in_use;
        self.space_left = space_left;
        self.total_size = total_size;
    }
} 