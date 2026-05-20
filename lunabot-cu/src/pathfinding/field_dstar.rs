use crate::pathfinding::OccupancyGrid;

use crate::pathfinding::flood_fill_escape;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};
type WorldCoord = (f32, f32);

type GridKey = (i32, i32);

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

    // Helper to check gradient from both maps (prioritize local)
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

    // if goal is not in bounds of the global map
    if !map.layout.is_in_bounds(goal[0], goal[1]) {
        eprintln!("[PathFinding] FAILED: Goal is out of bounds");
        return None;
    }

    let is_free = |x: f32, y: f32| -> bool {
        if let Some(grad) = get_gradient(x, y) {
            grad <= max_acceptable_gradient
        } else {
            false
        }
    };

    // Check if goal is free
    let goal_gradient = get_gradient(goal[0], goal[1]);
    let goal_is_free = is_free(goal[0], goal[1]);
    if !goal_is_free {
        eprintln!("[PathFinding] FAILED: Goal is blocked (gradient: {:?}, max acceptable: {})",
                 goal_gradient, max_acceptable_gradient);
        return None;
    }

    // Helper to convert world coordinates to grid keys
    // Prioritize local map, fall back to global map

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

    eprintln!("[PathFinding] FAILED: No path found (explored {} cells)", closed_set.len());
    None
}

pub fn find_path_dig_dijkstras(
    map: &OccupancyGrid,
    start: [f32; 2],
    max_acceptable_gradient: f32,
) -> Option<Vec<(f32, f32)>> {

    // Helper to check gradient from both maps (prioritize local)
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

    let is_free = |x: f32, y: f32| -> bool {
        if let Some(grad) = get_gradient(x, y) {
            grad <= max_acceptable_gradient
        } else {
            false
        }
    };

    // Helper to convert world coordinates to grid keys
    // Prioritize local map, fall back to global map

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

    let start_key = to_grid_key(start_world.0, start_world.1)?;

    g_score.insert(start_key, 0.0);
    open_set.push(Node {
        coord: neighbor_pos,
        0.0,
    });

    while let Some(Node { coord: current, .. }) = open_set.pop() {
        let Some(current_key) = to_grid_key(current.0, current.1) else {
            continue;
        };

        if to_world(current_key).0 < 2.25 {
            let mut path = vec![];
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
                let f_score = tentative_g;
                open_set.push(Node {
                    coord: neighbor_pos,
                    f_score,
                });
            }
        }
    }

    eprintln!("[PathFinding] FAILED: No path found (explored {} cells)", closed_set.len());
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
