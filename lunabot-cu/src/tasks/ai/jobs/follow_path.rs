use std::{f32, time::Duration};

use common::Steering;
use nalgebra::Vector2;
use simple_motion::StaticNode;
use tasker::tokio::{
    self,
    sync::{mpsc, watch},
};

use crate::tasks::ai::jobs::Job;

/// follows path fails if the robot fails to move significantly in stuck_timeout_secs
/// also could fail if this job is cancelled
pub fn follow_path_job(
    _stuck_timeout_secs: f32,
    chain: StaticNode,
    _path: Vec<Vector2<f32>>, // I just realized we're being given Vector2s, not f32 pairs. I'm a truly a fool.
) -> Job<Steering> {
    let (status_tx, status_rx) = watch::channel(bonsai_bt::Status::Running);
    let (output_tx, output_rx) = mpsc::channel(5);
    Job::spawn(
        async move {
            let _robot_isometry = chain.get_global_isometry();
            // The current target on the path being followed. Will move along the path continuously.
            let mut dot: (f32, f32) = (_robot_isometry.translation.x as f32, _robot_isometry.translation.y as f32);
            loop {
                let path: Vec<(f32, f32)> = Vec::new(); // TODO get path
                let dt: f32 = 0.1; // TODO determine or fix dt
                // In m/s. Will be determined by distance between robot and dot.
                let dot_speed = 0.1; // TODO determine by distance between robot and dot

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
                //   Move dot along closest segment
                dot = project_and_move_point_on_segment(
                    dot, 
                    (path[closest_segment_index], path[closest_segment_index+1]),
                    dot_speed * dt
                );

                // Calculate steering from dot and robot position
                //   TODO


                // OLD PLACEHOLDER CODE BELOW
                println!("sending steering value");
                // just sleep and then fail as an example but this loop should be following the path
                let _ = output_tx.send(Steering::new(1.0, 1.0, 5000.0)).await;
                // if you dont give a command to the vesc for more then ~1 second, as a saftey feature it will stop moving.
                // in order to get the robot to continuously move you have to continuously send commands.
                // For that reason, in this case it might be a good idea to keep track of the last known steering command from this job in the
                // match arm for this Action, and just always use the last known steering instead of "consuming" the steering commands requiring this
                // job to send more all the time.
                tokio::time::sleep(Duration::from_secs(1)).await;
                let _ = status_tx.send(bonsai_bt::Status::Failure);
                break;
            }
            let _ = output_tx.send(Steering::new(0.0, 0.0, 5000.0)).await;
            bonsai_bt::Status::Failure
        },
        status_rx,
        output_rx,
    )
}

/// Determines the minimum distance between any part of a given line segment to a given point.
fn distance_to_segment(start: (f32, f32), segment: ((f32, f32), (f32, f32))) -> f32 {
    let start_point_dist = pair_dist(start, segment.0);
    let end_point_dist = pair_dist(start, segment.1);
    
    let normalized_perpendicular_vector = pair_normalize((segment.0.1 - segment.1.1, segment.1.0 - segment.0.0));
    let parallel_dist = pair_dot_prod(normalized_perpendicular_vector, start).abs();

    start_point_dist.min(end_point_dist).min(parallel_dist)
}

/// Finds the closest point on a given line segment to a given point, then moves a given distance in the
/// direction of the segment. Pretends the line is infinite.
fn project_and_move_point_on_segment(start: (f32, f32), segment: ((f32, f32), (f32, f32)), movement: f32) -> (f32, f32) {
    // Use a dot product to project the given point onto the segment
    let normalized_parallel_vector = pair_normalize((segment.1.0 - segment.0.0, segment.0.1 - segment.1.1));
    let parallel_dist = movement + pair_dot_prod((start.0 - segment.0.0, start.1 - segment.0.1), normalized_parallel_vector);
    let parallel_projected_displacement = (normalized_parallel_vector.0 * parallel_dist, normalized_parallel_vector.1 * parallel_dist);

    (segment.0.0 + parallel_projected_displacement.0, segment.0.1 + parallel_projected_displacement.1)
}

/// Finds the distance between two points represented as tuples.
fn pair_dist(a: (f32, f32), b: (f32, f32)) -> f32 {
    let dif_x = a.0 - b.0;
    let dif_y = a.1 - b.1;

    pair_magnitude((dif_x, dif_y))
}

/// Finds the dot product between two points represented as tuples.
fn pair_dot_prod(a: (f32, f32), b: (f32, f32)) -> f32 {
    a.0*b.0 + a.1*b.1
}

/// Converts a tuple fector to a unit tuple vector.
fn pair_normalize(v: (f32, f32)) -> (f32, f32) {
    (v.0 / pair_magnitude(v), v.1 / pair_magnitude(v))
}

/// Finds the magnitude of a pair vector.
fn pair_magnitude(v: (f32, f32)) -> f32 {
    (v.0*v.0 + v.1*v.1).sqrt()
}