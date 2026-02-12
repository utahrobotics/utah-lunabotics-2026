use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::pathfinding::flood_fill_escape;
use crate::tasks::OccupancyGrid;

type WorldCoord = (f32, f32);

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum MapId {
    Local,
    Global,
}

type GridKey = (MapId, i32, i32);

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
/// If goal is out of the bounds of the global map, it will fail to find a path
/// prioritizes the local map as the source of ultimate truth, but falls back to the global map if a cell is unknown locally
/// Eventually may need to take in multiple local maps from different realsense devices
pub fn find_path_dstar(
    local_map: &OccupancyGrid,
    global_map: &OccupancyGrid,
    start: [f32; 2],
    goal: [f32; 2],
    max_acceptable_gradient: f32,
) -> Option<Vec<(f32, f32)>> {
    println!("[PathFinding] Starting pathfinding from {:?} to {:?}", start, goal);
    println!("[PathFinding] Max acceptable gradient: {}", max_acceptable_gradient);

    // Helper to check gradient from both maps (prioritize local)
    let get_gradient = |x: f32, y: f32| -> Option<f32> {
        if let Ok(Some(grad)) = local_map.gradient_closest_to(x, y) {
            return Some(grad);
        }
        global_map.gradient_closest_to(x, y).unwrap_or_default()
    };

    // Handle case where robot starts in obstacle or unknown area
    let mut initial_path = Vec::new();
    let start_gradient = get_gradient(start[0], start[1]);
    println!("[PathFinding] Start gradient: {:?}", start_gradient);

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
        println!("[PathFinding] Start is in obstacle or unknown, attempting flood fill escape");
        if let Some(valid_cell) =
            flood_fill_escape(local_map, global_map, start, max_acceptable_gradient, 200)
        {
            println!("[PathFinding] Found escape cell at {:?}", valid_cell);
            initial_path.push(valid_cell);
        } else {
            println!("[PathFinding] FAILED: Could not escape from start position");
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
    if !global_map.layout.is_in_bounds(goal[0], goal[1]) {
        println!("[PathFinding] FAILED: Goal is out of bounds");
        println!("[PathFinding] Global map bounds: min_x={}, max_x={}, min_y={}, max_y={}",
                 global_map.layout.min_x, global_map.layout.max_x,
                 global_map.layout.min_y, global_map.layout.max_y);
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
    println!("[PathFinding] Goal gradient: {:?}, Goal is free: {}", goal_gradient, goal_is_free);

    if !goal_is_free {
        println!("[PathFinding] FAILED: Goal is blocked (gradient: {:?}, max acceptable: {})",
                 goal_gradient, max_acceptable_gradient);
        return None;
    }

    // Determine which map to use for cell size
    let cell_size = global_map.layout.cell_size;

    // Helper to convert world coordinates to grid keys
    // Prioritize local map, fall back to global map
    let to_grid_key = |x: f32, y: f32| -> Option<GridKey> {
        if let Ok((cx, cy)) = local_map.world_to_cell(x, y) {
            Some((MapId::Local, cx as i32, cy as i32))
        } else {
            global_map
                .world_to_cell(x, y)
                .ok()
                .map(|(cx, cy)| (MapId::Global, cx as i32, cy as i32))
        }
    };

    // Helper to convert grid key back to world coordinates
    let to_world = |key: GridKey| -> Option<WorldCoord> {
        match key.0 {
            MapId::Local => local_map.cell_to_world(key.1 as usize, key.2 as usize).ok(),
            MapId::Global => global_map
                .cell_to_world(key.1 as usize, key.2 as usize)
                .ok(),
        }
    };

    // Neighbors are generated in world space and then converted back to grid keys
    // This allows transitions between local and global maps
    let get_neighbors_for_key = |key: GridKey| -> Vec<GridKey> {
        let mut neighbors = Vec::new();

        let Some(current_world) = to_world(key) else {
            return neighbors;
        };

        // iterate through neighbors in world space by stepping by cell_size
        for dx in -1..=1 {
            for dy in -1..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }

                let neighbor_x = current_world.0 + (dx as f32) * cell_size;
                let neighbor_y = current_world.1 + (dy as f32) * cell_size;

                // conv back to grid key - this will automatically choose
                // local map if available, otherwise global map
                if let Some(neighbor_key) = to_grid_key(neighbor_x, neighbor_y) {
                    neighbors.push(neighbor_key);
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

            println!("[PathFinding] SUCCESS: Found path with {} waypoints", path.len());
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

    println!("[PathFinding] FAILED: No path found (explored {} cells)", closed_set.len());
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
