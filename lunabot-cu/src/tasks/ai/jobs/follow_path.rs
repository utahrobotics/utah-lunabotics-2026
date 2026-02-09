use std::{f32::{self}, time::Duration};

use common::Steering;
use nalgebra::Vector2;
use simple_motion::StaticNode;
use tasker::tokio::{
    self,
    sync::{mpsc, watch},
};

use crate::tasks::ai::jobs::Job;

// Distance between wheels, in m
const WHEEL_BASE_SIZE: f32 = 0.6 * 5.0; // TODO fill in
const FOLLOW_SPEED_FACTOR: f32 = 0.15;
const MAX_DOT_SPEED: f32 = 0.2; // In m/s
const DOT_SPEED_FACTOR: f32 = 0.005; // In m/s // TODO test and adjust

/// follows path ~~fails if the robot fails to move significantly in stuck_timeout_secs~~ (TODO)
/// also could fail if this job is cancelled
pub fn follow_path_job(
    _stuck_timeout_secs: f32, // Currently not used
    chain: StaticNode,
    path: Vec<Vector2<f32>>,
) -> Job<Steering> {
    let (status_tx, status_rx) = watch::channel(bonsai_bt::Status::Running);
    let (output_tx, output_rx) = mpsc::channel(5);
    Job::spawn(
        async move {
            // The current target on the path being followed. Will move along the path continuously.
            let mut dot: Vector2<f32> = chain.get_global_isometry().translation.vector.xy().cast();
            let mut print_accumulator: f32 = 0.0; // FOR DEBUG USE
            loop {
                let _robot_isometry = chain.get_global_isometry();
                // Flatten and set up
                let robot_pos = _robot_isometry.translation.vector.xy().cast();
                let robot_angle = _robot_isometry.rotation.euler_angles().2 as f32;
                let error = dot - robot_pos;

                let dt: f32 = 0.05; // TODO determine or fix dt

                // In m/s. Moves faster when robot is closer, by inverse square law.
                let dot_speed = (DOT_SPEED_FACTOR / error.norm_squared()).min(MAX_DOT_SPEED);

                // Update dot location
                //   Find closest segment
                let mut min_dist = f32::INFINITY;
                let mut closest_segment_index: usize = 0;
                for i in 0..path.len()-1 {
                    let dist = distance_to_segment(dot, (path[i], path[i+1]));
                    if dist < min_dist {
                        min_dist = dist;
                        closest_segment_index = i;
                    }
                }

                if closest_segment_index < path.len() - 1 { // Hacky way to end path
                    //   Move dot along closest segment
                    dot = project_and_move_point_on_segment(
                        dot, 
                        (path[closest_segment_index], path[closest_segment_index+1]),
                        dot_speed * dt
                    );
                }

                // Calculate steering from dot and robot position
                //   Some relevant math:
                //     https://www.desmos.com/calculator/pbk1kdxg5z
                //     https://www.desmos.com/geometry/emcgvxxrg5
                //   Determine radius of turn, so that turn intersects dot, and
                //   aligns with current robot angle.
                let target_distance = error.norm();
                let error_angle = error.y.atan2(error.x);
                let angle_difference = error_angle - robot_angle;
                let radius = target_distance / (2.0 * angle_difference.sin()); // Check if this has a sign error

                print_accumulator += dt; // DEBUG
                // Only move if the dot is far enough away
                let steering = if error.norm_squared() > 0.1 {
                    //   Radius is proportional to the ratio of velocity to angular velocity:
                    //   https://www.desmos.com/calculator/f7grn652s4
                    // TODO Fix problems with dot being behind bot
                    let velocity = FOLLOW_SPEED_FACTOR * target_distance; // will be fixed by normalization // currently always goes max speed
                    let turning = velocity * WHEEL_BASE_SIZE * 0.5 / radius;

                    // DEBUG
                    if print_accumulator > 1.0 {
                        println!("Robot: ({}, {})\t Dot: ({}, {})\t Error: {}\t Control: (^ = {}, <-> = {})", 
                            robot_pos.x, robot_pos.y, 
                            dot.x, dot.y, 
                            target_distance,
                            velocity, 
                            turning
                        );
                        print_accumulator = 0.0;
                    }

                    Steering::new_ik(velocity as f64, turning as f64, 5000.0)
                } else {
                    Steering::new(0.0, 0.0, 5000.0)
                };


                let _ = output_tx.send(steering).await;
                tokio::time::sleep(Duration::from_secs_f32(dt)).await;
                let _ = status_tx.send(bonsai_bt::Status::Running); // This was used without full understanding - H
            }
        },
        status_rx,
        output_rx,
    )
}

/// Determines the minimum distance between any part of a given line segment to a given point.
fn distance_to_segment(point: Vector2<f32>, segment: (Vector2<f32>, Vector2<f32>)) -> f32 {
    // Move to frame where the start point is at the origin
    let segment_vec = segment.1 - segment.0;
    let point_from_start = point - segment.0;

    let start_point_dist = point_from_start.norm();
    let end_point_dist = (point - segment.1).norm();
    
    // If the point is between the two points (in the perpendicular projection of the segment),
    // return the parallel distance
    let dist_along_segment = point_from_start.dot(&segment_vec.normalize());
    if dist_along_segment > 0.0 || dist_along_segment < segment_vec.norm() {
        // a X v = |a| |b| cos(theta), but length of line segment shouldn't matter,
        // so divide by |b|.
        return vector_2_cross(segment_vec, point_from_start) / segment_vec.norm();
    }

    return start_point_dist.min(end_point_dist);
}

/// Finds the closest point on a given line segment to a given point, then moves a given distance in the
/// direction of the segment. Pretends the line is infinite.
fn project_and_move_point_on_segment(point: Vector2<f32>, segment: (Vector2<f32>, Vector2<f32>), movement: f32) -> Vector2<f32> {
    // Move to frame where the start point is at the origin
    let segment_vec = segment.1 - segment.0;
    let point_from_start = point - segment.0;
    // Use a dot product to project the given point onto the segment
    let projection_distance = point_from_start.dot(&segment_vec.normalize());
    return (projection_distance + movement) * segment_vec.normalize() + segment.0;
}

fn vector_2_cross(a: Vector2<f32>, b: Vector2<f32>) -> f32 {
    return a.x * b.y - a.y * b.x;
}