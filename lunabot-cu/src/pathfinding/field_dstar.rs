use common::Steering;

use crate::pathfinding::OccupancyGrid;

use crate::pathfinding::flood_fill_escape;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
use std::f32::consts::FRAC_PI_3;

const SQRT_3: f32 = 1.7320507764816284;
const SQRT_3_INV: f32 = 1.0 / SQRT_3;
const GRID_SPACE: f32 = 0.1;
const ANGLE_SPACE: f32 = FRAC_PI_3;

const STEERING_POWER: f64 = 2000.0;

type WorldCoord = (f32, f32);
type WorldPose = (f32, f32, f32); // x, y, theta (rad) (+x = 0)

type GridKey = (i32, i32);
type GridPose = (i32, i32, i32);

#[derive(Clone, Copy)]
struct Node {
    coord: WorldCoord,
    f_score: f32,
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f_score == other.f_score
            && (self.coord.0 - other.coord.0).abs() < 0.001
            && (self.coord.1 - other.coord.1).abs() < 0.001
    }
}

impl Eq for Node {}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .f_score
            .partial_cmp(&self.f_score)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Node) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// If goal is an obstacle, it will not pathfind to it
/// If goal is out of the bounds of the map, it will fail to find a path
/// Expects a single pre-expanded combined map (local merged into global, then obstacle-expanded)
pub fn find_path_dstar(
    map: &OccupancyGrid,
    start: [f32; 2],
    goal: [f32; 2],
    max_acceptable_gradient: f32,
) -> Option<Vec<(f32, f32)>> {
    // Helper to check gradient
    let get_gradient = |x: f32, y: f32| -> Option<f32> {
        Some(
            map.gradient_closest_to(x, y)
                .unwrap_or_default()
                .unwrap_or(f32::MIN),
        )
    };

    // Handle case where robot starts in obstacle or unknown area
    let mut initial_path = Vec::new();
    if get_gradient(start[0], start[1])
        .map(|val| {
            if val > max_acceptable_gradient {
                None
            } else {
                Some(val)
            }
        })
        .flatten()
        .is_none()
    {
        if let Some(valid_cell) =
            flood_fill_escape(map, start, max_acceptable_gradient, 200)
        {
            initial_path.push(valid_cell);
        } else {
            return None;
        }
    }

    let start = if initial_path.is_empty() {
        start
    } else {
        let end = *initial_path.last().unwrap();
        [end.0, end.1]
    };

    if !map.layout.is_in_bounds(goal[0], goal[1]) {
        return None;
    }

    let is_free = |x: f32, y: f32| -> bool {
        if let Some(grad) = get_gradient(x, y) {
            grad <= max_acceptable_gradient
        } else {
            false
        }
    };

    let to_grid_key = |x: f32, y: f32| -> Option<GridKey> {
        map.world_to_cell(x, y)
            .ok()
            .map(|(cx, cy)| (cx as i32, cy as i32))
    };

    let to_world = |key: GridKey| -> Option<WorldCoord> {
        map.cell_to_world(key.0 as usize, key.1 as usize).ok()
    };

    let cells_x = map.cells_x() as i32;
    let cells_y = map.cells_y() as i32;

    let get_neighbors_for_key = |key: GridKey| -> Vec<GridKey> {
        let mut neighbors = Vec::new();
        for dx in -1..=1i32 {
            for dy in -1..=1i32 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let nx = key.0 + dx;
                let ny = key.1 + dy;
                if nx >= 0 && ny >= 0 && nx < cells_x && ny < cells_y {
                    neighbors.push((nx, ny));
                }
            }
        }
        neighbors
    };

    let mut open_set = BinaryHeap::new();
    let mut came_from: HashMap<GridKey, GridKey> = HashMap::new();
    let mut g_score: HashMap<GridKey, f32> = HashMap::new();
    let mut closed_set: HashSet<GridKey> = HashSet::new();

    let start_world = (start[0], start[1]);
    let goal_world = (goal[0], goal[1]);

    let start_key = to_grid_key(start_world.0, start_world.1)?;
    let goal_key = to_grid_key(goal_world.0, goal_world.1)?;

    g_score.insert(start_key, 0.0);
    let h_start = heuristic(start_world, goal_world);
    open_set.push(Node {
        coord: start_world,
        f_score: h_start,
    });

    while let Some(Node { coord: current, .. }) = open_set.pop() {
        let Some(current_key) = to_grid_key(current.0, current.1) else {
            continue;
        };

        if current_key == goal_key {
            let mut path = vec![goal_world];
            let mut curr_key = current_key;

            while curr_key != start_key {
                if let Some(curr_world) = to_world(curr_key) {
                    path.push(curr_world);
                }
                if let Some(&prev_key) = came_from.get(&curr_key) {
                    curr_key = prev_key;
                } else {
                    break;
                }
            }
            path.push(start_world);
            path.reverse();

            return Some(path);
        }

        if closed_set.contains(&current_key) {
            continue;
        }
        closed_set.insert(current_key);

        let current_g = *g_score.get(&current_key).unwrap_or(&f32::INFINITY);

        for neighbor_key in get_neighbors_for_key(current_key) {
            if closed_set.contains(&neighbor_key) {
                continue;
            }

            let Some(neighbor_pos) = to_world(neighbor_key) else {
                continue;
            };

            if !is_free(neighbor_pos.0, neighbor_pos.1) {
                continue;
            }

            let move_dist = distance(current, neighbor_pos);
            let tentative_g = current_g + move_dist;

            let existing_g = *g_score.get(&neighbor_key).unwrap_or(&f32::INFINITY);

            if tentative_g < existing_g {
                came_from.insert(neighbor_key, current_key);
                g_score.insert(neighbor_key, tentative_g);
                let f_score = tentative_g + heuristic(neighbor_pos, goal_world);
                open_set.push(Node {
                    coord: neighbor_pos,
                    f_score,
                });
            }
        }
    }

    None
}

fn heuristic(a: WorldCoord, b: WorldCoord) -> f32 {
    distance(a, b)
}

fn distance(a: WorldCoord, b: WorldCoord) -> f32 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    (dx * dx + dy * dy).sqrt()
}

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
) -> impl Fn(WorldPose) -> Steering {

    let goal_index = cartesian_to_index(goal);

    let mut frontier: BinaryHeap<AStarKey> = BinaryHeap::new();
    let mut visited: HashSet<GridPose> = HashSet::new();
    let mut policy: HashMap<GridPose, Steering> = HashMap::new();
    
    frontier.push(AStarKey { pose: goal_index, cost: 0.0 });

    while !frontier.is_empty() {
        let next = frontier.pop().unwrap();
        visited.insert(next.pose);

        for neighbor in iso_neighbors(map, next.pose) {
            if visited.contains(&neighbor.2) {
                continue;
            }

            // Is it in the frontier?
            if frontier.iter().map(|x| {x.pose}).any(|x| {x == next.pose}) {

            }

            // But wait, the actions are associated with paths, not points. What do you do?
        }
    }


    return |s| {Steering::new(0.0, 0.0, 0.0)}
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

/// Returns: (Steering, cost weight (path length), end state)
fn iso_neighbors(map: &OccupancyGrid, pose: GridPose) -> Vec<(Steering, f32, GridPose)> {
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

        // TODO steering inputs
        out.push((Steering::new_ik(0.0, 0.0, 0.0), 1.0, result));
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

fn is_iso_point_in_bounds(map: &OccupancyGrid, pose: GridPose) -> bool {
    let cartesian = index_to_cartesian(pose);
    map.layout.is_in_bounds(cartesian.0, cartesian.1)
}