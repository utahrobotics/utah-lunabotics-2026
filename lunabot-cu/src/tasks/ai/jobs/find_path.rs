use crate::{pathfinding::{OccupancyGrid, sampling::find_goal_in}, tasks::ai::behaviors::autonomy::navigate::NavigationGoal};
use nalgebra::Vector2;
use rerun::Vec2D;
use tasker::tokio::sync::mpsc;

use crate::{
    pathfinding::field_dstar::find_path_dstar,
    rerun_viz::RECORDER,
    tasks::{ai::jobs::Job, realsense_occupancy_grid::GLOBAL_MAP},
};

/// IMPORTANT: this job should not take more than a few ms, because since it has access to the global map's read guard,
/// it could cause delays in other tasks that need the global map if it takes too long.
/// Merges local map into global, expands obstacles on the combined map, then runs D* pathfinding.
/// if the robot start position is in an unknown or obstacle, itll find its way out of that area first by searching around with flood_fill_escape to find a near free space and then start from there instead
/// FAILS IF:
/// 2. there isnt a path to be found from start to end
pub fn find_path_job(
    latest_local_map: OccupancyGrid,
    start: Vector2<f32>,
    max_acceptable_gradient_expander: f32,
    max_acceptable_gradient_pathfinder: f32,
    robot_radius: f32,
    goal: NavigationGoal
) -> Job<Vec<Vector2<f32>>, ()> {
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

                // Merge local into global before expanding so obstacle inflation
                // propagates seamlessly across the local/global boundary.
                let mut combined = global_map_guard.clone();
                let _ = latest_local_map.append_to(&mut combined);
                let Some(expanded) =
                    combined.expand_obstacles(robot_radius, max_acceptable_gradient_expander)
                else {
                    return None;
                };
                let (center, hw, hh) = goal.to_center_and_halfsizes();
                let end = find_goal_in(&expanded, center, hw, hh, max_acceptable_gradient_expander).ok()?;
                find_path_dstar(
                    &expanded,
                    [start.x, start.y],
                    [end.x, end.y],
                    max_acceptable_gradient_pathfinder,
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
                bonsai_bt::Status::Success
            } else {
                bonsai_bt::Status::Failure
            };
            result
        },
        output_rx,
        None,
    )
}
