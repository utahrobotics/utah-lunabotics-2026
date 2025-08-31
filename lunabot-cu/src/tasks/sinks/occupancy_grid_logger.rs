use common::{Occupancy, THALASSIC_CELL_SIZE, THALASSIC_WIDTH};
use cu29::prelude::*;
use iceoryx_types::IceoryxOccupancyGrid;
use rerun::{Color, Points3D};

use crate::rerun_viz::RECORDER;

pub struct OccupancyGridSink {}

impl Freezable for OccupancyGridSink {}

impl CuSinkTask for OccupancyGridSink {
    type Input<'m> = input_msg!(IceoryxOccupancyGrid);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(grid) = input.payload() {
            let mut positions = Vec::new();
            let mut colors = Vec::new();
            let mut labels = Vec::new();
            for (point, score) in grid.data.iter().enumerate().map(|(i, score)| {
                let (x, y) = index_to_xy(i);
                (
                    rerun::Position3D::new(
                        x as f32 * THALASSIC_CELL_SIZE,
                        y as f32 * THALASSIC_CELL_SIZE,
                        0.0,
                    ),
                    Occupancy(*score),
                )
            }) {
                positions.push(point);
                labels.push(score.0.to_string());
                let color = match score.0 {
                    0 => Color::from_rgb(128, 128, 128), // Grey for unknown
                    1..=100 => {
                        // Gradient from green (1) to red (100)
                        let t = (score.0 - 1) as f32 / 99.0; // Normalize to 0-1
                        let r = (255.0 * t) as u8;
                        let g = (255.0 * (1.0 - t)) as u8;
                        let b = 0u8;
                        Color::from_rgb(r, g, b)
                    }
                    _ => Color::from_rgb(255, 0, 0), // Fallback to red for any value > 100
                };

                colors.push(color);
            }
            let _ = RECORDER.get().unwrap().recorder.log(
                "occupancy",
                &Points3D::new(positions)
                    .with_colors(colors)
                    .with_labels(labels),
            );
        }

        Ok(())
    }
}

fn index_to_xy(index: usize) -> (usize, usize) {
    (
        index % THALASSIC_WIDTH as usize,
        index / THALASSIC_WIDTH as usize,
    )
}
