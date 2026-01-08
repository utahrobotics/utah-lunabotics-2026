use nalgebra::Vector2;
use rerun::Vec2D;
use tasker::tokio::sync::{mpsc, watch};

use crate::{
    pathfinding::field_dstar::find_path_dstar,
    rerun_viz::RECORDER,
    tasks::{OccupancyGrid, ai::jobs::Job, realsense_occupancy_grid::GLOBAL_MAP},
};

/// IMPORTANT: this job should not take more than a few ms, because since it has access to the global map's read guard,
/// it could cause delays in other tasks that need the global map if it takes too long.
/// Uses D* to navigate from start to end
/// prioritizes the local map as the source of ultimate truth, but falls back to the global map if a cell is unknown locally
/// Eventually may need to take in multiple local maps from different realsense devices
pub fn find_path_job(
    latest_local_map: OccupancyGrid,
    start: Vector2<f32>,
    end: Vector2<f32>,
) -> Job<Vec<Vector2<f32>>> {
    let (status_tx, status_rx) = watch::channel(bonsai_bt::Status::Running);
    let (output_tx, output_rx) = mpsc::channel(5);

    Job::spawn(
        async move {
            if let Some(rec) = RECORDER.get() {
                let _ = rec.recorder.log(
                    "ai/position",
                    &rerun::Points2D::new([Vec2D::new(start.x, start.y)])
                        .with_colors([rerun::Color::from_rgb(0, 255, 0)]),
                );
            }
            let path_result = tasker::tokio::task::spawn_blocking(move || {
                let global_map_guard = if let Some(global_map) = GLOBAL_MAP.get() {
                    global_map.read().ok()
                } else {
                    return None;
                };

                let Some(global_map_guard) = global_map_guard else {
                    return None;
                };

                find_path_dstar(
                    &latest_local_map,
                    &*global_map_guard,
                    [start.x, start.y],
                    [end.x, end.y],
                    0.3, // max_acceptable_gradient
                )
            })
            .await
            .unwrap_or(None);

            let result = if let Some(path) = path_result {
                if let Some(rec) = RECORDER.get() {
                    let _ = rec.recorder.log(
                        "ai/calculated_path",
                        &rerun::LineStrips2D::new([path
                            .iter()
                            .map(|p| Vec2D::new(p.0, p.1))
                            .collect::<Vec<_>>()])
                        .with_colors([rerun::Color::from_rgb(0, 200, 0)])
                        .with_draw_order(100.0),
                    );
                }

                let vector_path: Vec<Vector2<f32>> =
                    path.into_iter().map(|(x, y)| Vector2::new(x, y)).collect();

                let _ = output_tx.send(vector_path).await;
                let _ = status_tx.send(bonsai_bt::Status::Success);
                bonsai_bt::Status::Success
            } else {
                let _ = status_tx.send(bonsai_bt::Status::Failure);
                bonsai_bt::Status::Failure
            };
            result
        },
        status_rx,
        output_rx,
    )
}
