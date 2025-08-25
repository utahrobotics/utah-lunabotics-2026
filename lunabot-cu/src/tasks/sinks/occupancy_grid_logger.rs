use common::{Occupancy, THALASSIC_CELL_COUNT, THALASSIC_CELL_SIZE, THALASSIC_WIDTH};
use cu29::prelude::*;
use iceoryx_types::IceoryxOccupancyGrid;
use rerun::{Color, Points3D, Position3D};

use crate::rerun_viz::RECORDER;


pub struct OccupancyGridSink {

}

impl Freezable for OccupancyGridSink {}


impl CuSinkTask for OccupancyGridSink {
    type Input<'m> = input_msg!(IceoryxOccupancyGrid);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self>
    where
        Self: Sized {
        Ok(Self{})
    }

    fn process<'i>(&mut self, _clock: &RobotClock, input: &Self::Input<'i>) -> CuResult<()> {
        if let Some(grid) = input.payload() {
            let mut positions = Vec::new();
            let mut colors = Vec::new();
            for (point, score) in grid.data.iter().enumerate().map(|(i, score)| {
                let (x, y) = index_to_xy(i);
                (rerun::Position3D::new(x as f32 * THALASSIC_CELL_SIZE, y as f32 * THALASSIC_CELL_SIZE, 0.0), Occupancy(*score))
            }) {
                positions.push(point);
                colors.push(match score.occupancy_type() {
                    common::OccupancyType::Occupied(score) => {
                        Color::from_rgb(255, 0, 0)
                    },
                    common::OccupancyType::Free(score) => {
                        Color::from_rgb(0, 255, 0)
                    },
                    common::OccupancyType::Unknown => {
                        Color::BLACK
                    },
                })
            }
            RECORDER.get().unwrap().recorder.log("occupancy", &Points3D::new(positions).with_colors(colors));
        }

        Ok(())
    }
}

fn index_to_xy(index: usize) -> (usize, usize) {
    (index % THALASSIC_WIDTH as usize,index / THALASSIC_WIDTH as usize)
}