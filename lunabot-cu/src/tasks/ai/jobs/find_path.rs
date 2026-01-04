use nalgebra::Vector2;

use crate::tasks::{OccupancyGrid, ai::jobs::Job};

pub fn find_path_job(latest_local_map: OccupancyGrid) -> Job<Vec<Vector2<f64>>> {

    if let Some(ref map) = blackboard.latest_obstacle_map {
                    let translation = blackboard.kinematic_root.get_global_isometry().translation;
                    if let Some(path) = find_path(
                        map,
                        [translation.x as f32, translation.y as f32],
                        PATHFINDING_GOAL,
                        0.3,
                        map.cell_size * 2.,
                        // FIXME: tune this parameter
                        1000,
                    ) && let Some(rec) = RECORDER.get()
                    {
                        let _ = rec.recorder.log(
                            "ai/calculated_path",
                            &rerun::LineStrips3D::new(&[path
                                .iter()
                                .map(|p| Vec3D::new(p.0 as f32, p.1 as f32, 1.0))
                                .collect::<Vec<_>>()])
                            .with_colors(vec![rerun::Color::from_rgb(0, 200, 0)]),
                        );
                        Success
                    } else {
                        Failure
                    }
                } else {
                    Failure
                } // if let Some(ref map) = blackboard.latest_obstacle_map {
                    let translation = blackboard.kinematic_root.get_global_isometry().translation;
                    if let Some(path) = find_path(
                        map,
                        [translation.x as f32, translation.y as f32],
                        PATHFINDING_GOAL,
                        0.3,
                        map.cell_size * 2.,
                        // FIXME: tune this parameter
                        1000,
                    ) && let Some(rec) = RECORDER.get()
                    {
                        let _ = rec.recorder.log(
                            "ai/calculated_path",
                            &rerun::LineStrips3D::new(&[path
                                .iter()
                                .map(|p| Vec3D::new(p.0 as f32, p.1 as f32, 1.0))
                                .collect::<Vec<_>>()])
                            .with_colors(vec![rerun::Color::from_rgb(0, 200, 0)]),
                        );
                        Success
                    } else {
                        Failure
                    }
                } else {
                    Failure
                }

    todo!()

}