use crate::pathfinding::OccupancyGrid;
use crate::rerun_viz::{RECORDER, RecorderData};

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::f32::consts::{FRAC_PI_2, FRAC_PI_3, PI};
use std::io;

// Pure constants
const SQRT_3: f32 = 1.7320507764816284;
const SQRT_3_INV: f32 = 1.0 / SQRT_3;
const ANGLE_SPACE: f32 = FRAC_PI_3;

// Adjustable constants
const GRID_SPACE: f32 = 0.1;
const GRADIENT_COST_FACTOR: f32 = 0.5;

type WorldPose = (f32, f32, f32); // x, y, theta (rad) (+x = 0)
type GridPose = (i32, i32, i32); // u1, u2, rotation (increments)

#[derive(Clone, Copy)]
struct AStarKey {
    pose: GridPose,
    cost: f32,
}

impl Ord for AStarKey {
    fn cmp(&self, other: &Self) -> Ordering {
        self.cost.total_cmp(&other.cost)
    }
}

// Rust has a RICH TYPE SYSTEM!!!!
impl PartialOrd for AStarKey {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// Rust, you literally have derives for this.
impl PartialEq for AStarKey {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other).is_eq()
    }
}

// Come on Rust, no need to overthink this.
impl Eq for AStarKey {}


/// Represents the action that should be performed at a given node.
#[derive(Clone, Copy, Debug)]
pub struct ActionControl {
    /// omega / (v + omega)
    ///   where omega is turn rate and v is forward velocity
    /// 1.0 is full turning CCW, 0.0 is full ahead, and -1.0
    /// is full turning CW
    pub turn_percent: f32,
    pub forward: bool,
}

#[derive(Debug, Clone)]
pub struct NavigationPolicy {
    policy: HashMap<GridPose, ActionControl>,
    goal: GridPose,
}

impl NavigationPolicy {
    pub fn action_closest_to_pose(&self, &pose: &WorldPose) -> Option<ActionControl> {
        let closest = cartesian_to_index(pose);
        self.policy.get(&closest).map(|v| *v)
    }

    pub fn full_goal_dist(&self, pose: WorldPose) -> f32 {
        let goal_pose = index_to_cartesian(self.goal);

        let angle_dif = modulo_f(pose.0 - goal_pose.0, PI);
        let angle_dif = if angle_dif > FRAC_PI_2 {PI - angle_dif} else {angle_dif};

        return ((pose.0 - goal_pose.0).powi(2) + (pose.1 - goal_pose.1).powi(2) + (angle_dif).powi(2)).sqrt()
    }
}


/// Finds a policy (what to do in any state) from an obstacle map, and
/// a goal. Uses reverse A*, which is fairly inefficient, on a lattice
/// using motion primitives.
/// ## Improvements:
/// * Switch to D* Lite
/// * Allow a goal set instead of goal point
pub fn find_policy(
    map: &OccupancyGrid,
    goal: WorldPose,
    max_acceptable_gradient: f32,
) -> Result<NavigationPolicy, io::Error> {

    let goal_index = cartesian_to_index(goal);

    let mut frontier: BinaryHeap<AStarKey> = BinaryHeap::new();
    let mut frontier_set: HashMap<GridPose, f32> = HashMap::new();
    let mut visited: HashSet<GridPose> = HashSet::new();
    let mut policy: HashMap<GridPose, ActionControl> = HashMap::new();
    
    frontier.push(AStarKey { pose: goal_index, cost: 0.0 });
    frontier_set.insert(goal_index, 0.0);

    while !frontier.is_empty() {
        let next = frontier.pop().unwrap();
        frontier_set.remove(&next.pose);

        visited.insert(next.pose);

        for neighbor in iso_neighbors(map, next.pose) {
            if visited.contains(&neighbor.2) {
                continue;
            }

            let (world_x, world_y, _) = index_to_cartesian(neighbor.2);
            let gradient = map.gradient_around(world_x, world_y, 1).ok().flatten().unwrap_or(0.0);
            let gradient_cost = if gradient > max_acceptable_gradient { f32::INFINITY } else { gradient * GRADIENT_COST_FACTOR };
            let end_cost = next.cost + neighbor.1 + gradient_cost;

            // Is it in the frontier? If so, update it iff we just found a better path.
            if let Some(&current_cost) = frontier_set.get(&neighbor.2) {
                if current_cost > end_cost {
                    policy.insert(neighbor.2, neighbor.0);

                    frontier.retain(|x| {x.pose != neighbor.2});
                    frontier.push(AStarKey { pose: neighbor.2, cost: end_cost });
                    frontier_set.insert(neighbor.2, end_cost);
                }
                continue;
            }

            policy.insert(neighbor.2, neighbor.0);

            frontier.push(AStarKey { pose: neighbor.2, cost: end_cost });
            frontier_set.insert(neighbor.2, end_cost);
        }
    }

    return Ok(NavigationPolicy{ policy, goal: goal_index })
}

fn index_to_cartesian(grid_pose: GridPose) -> WorldPose {
    let position = iso_to_cartesian(grid_pose.0 as f32 * GRID_SPACE, grid_pose.1 as f32 * GRID_SPACE);
    
    (position.0, position.1, grid_pose.2 as f32 * ANGLE_SPACE)
}

fn cartesian_to_index(pose: WorldPose) -> GridPose {
    let position = cartesian_to_iso(pose.0, pose.1);
    
    (
        (position.0 / GRID_SPACE).round() as i32,
        (position.1 / GRID_SPACE).round() as i32,
        (pose.2 / ANGLE_SPACE).round() as i32,
    )
}

fn iso_to_cartesian(u_1: f32, u_2: f32) -> (f32, f32) {
    (
        u_1 * SQRT_3 * 0.5,
        u_2 + u_1 * 0.5,
    )
}

fn cartesian_to_iso(x: f32, y: f32) -> (f32, f32) {
    (
        2.0 * SQRT_3_INV * x,
        y - SQRT_3_INV * x,
    )
}

/// Returns: (ActionControl, cost weight (path length), end state)
fn iso_neighbors(map: &OccupancyGrid, pose: GridPose) -> Vec<(ActionControl, f32, GridPose)> {
    let motion_primitives = [
        (0,1,0),

        (-1,2,1),
        (-1,1,2),
        (-2,2,2),
        (-2,1,3),

        (-1,-1,-1),
        (-1,0,-2),
        (-2,0,-2),
        (-2,1,-3),

        (0,-1,0),

        (1,-2,1),
        (1,-1,2),
        (2,-2,2),
        (2,-1,3),

        (1,1,-1),
        (1,0,-2),
        (2,0,-2),
        (2,-1,-3),

        (0, 0, 1),
        (0, 0, -1),
    ];

    let mut out = vec![];

    for motion in motion_primitives {
        let dif = iso_grid_rotate(motion, pose.2);
        // We've rotated the little difference on the end, the rotation doesn't need to be added again
        let result = (pose.0 + dif.0, pose.1 + dif.1, dif.2);

        if is_iso_point_in_bounds(map, result) {
            // Determine steering
            let angle = dif.2 as f32 * ANGLE_SPACE;
            let (x, y, _) = index_to_cartesian(dif);
            let arc_length = 
                if angle.abs() < 1e-16 {
                    (x.powi(2) + y.powi(2)).sqrt() * angle * 0.5 / (0.5 * angle).sin()
                } else {
                    y.abs()
                }
            ;
            let turn_percent = -angle / (arc_length + angle.abs());
            let forward = y >= 0.0;
            
            out.push((ActionControl { turn_percent, forward}, angle.abs() + arc_length.abs(), result));
        }
    }

    out
}

fn iso_grid_rotate(pose: GridPose, rotation: i32) -> GridPose {
    let rotation = modulo(rotation, 6);

    return match rotation {
        0 => pose,
        1 => (        -pose.1,  pose.0 +pose.1, modulo(pose.2 + rotation, 6)),
        2 => (-pose.0 -pose.1,  pose.0        , modulo(pose.2 + rotation, 6)),
        3 => (-pose.0        ,         -pose.1, modulo(pose.2 + rotation, 6)),
        4 => ( pose.0 +pose.1, -pose.0        , modulo(pose.2 + rotation, 6)),
        5 => (         pose.1, -pose.0 -pose.1, modulo(pose.2 + rotation, 6)),
        _ => panic!("rotation was not in [0,5] after modulo.")
    };
}

fn modulo(a: i32, b: i32) -> i32 {
    ((a%b) + b) % b
}

fn modulo_f(a: f32, b: f32) -> f32 {
    ((a%b) + b) % b
}

fn is_iso_point_in_bounds(map: &OccupancyGrid, pose: GridPose) -> bool {
    let cartesian = index_to_cartesian(pose);
    map.layout.is_in_bounds(cartesian.0, cartesian.1)
}