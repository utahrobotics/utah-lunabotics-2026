use crate::tasks::OccupancyGrid;

/// used for when the robot appears to be stuck in unknown or obstacle space
/// returns the nearest free space (by Euclidean distance)
/// Searches both local and global map
pub fn flood_fill_escape(
    local_map: &OccupancyGrid,
    global_map: &OccupancyGrid,
    start: [f32; 2],
    max_acceptable_gradient: f32,
    num_max_try: usize,
) -> Option<(f32, f32)> {
    let check_map = |map: &OccupancyGrid, cell_x: usize, cell_y: usize| -> Option<(f32, f32)> {
        let grad = map.gradient_around_cell(cell_x, cell_y, 3).ok()??;
        if grad <= max_acceptable_gradient {
            map.cell_to_world(cell_x, cell_y).ok()
        } else {
            None
        }
    };

    let local_start = local_map.world_to_cell(start[0], start[1]);
    let global_start = global_map.world_to_cell(start[0], start[1]);

    if local_start.is_err() && global_start.is_err() {
        return None;
    }

    let mut layer = 1;
    loop {
        if layer > num_max_try as isize {
            break;
        }

        let mut candidates = Vec::new();

        //  in local map if we have a starting cell
        if let Ok((cell_x, cell_y)) = local_start {
            for i in -layer..=layer {
                for j in -layer..=layer {
                    if i.abs() != layer && j.abs() != layer {
                        continue;
                    }
                    let x = cell_x as isize + i;
                    let y = cell_y as isize + j;
                    if x < 0 || y < 0 {
                        continue;
                    }

                    if let Some(world_pos) = check_map(local_map, x as usize, y as usize) {
                        let dist_sq =
                            (world_pos.0 - start[0]).powi(2) + (world_pos.1 - start[1]).powi(2);
                        candidates.push((world_pos, dist_sq));
                    }
                }
            }
        }

        //  in global map if we have a starting cell

        if let Ok((cell_x, cell_y)) = global_start {
            for i in -layer..=layer {
                for j in -layer..=layer {
                    if i.abs() != layer && j.abs() != layer {
                        continue;
                    }
                    let x = cell_x as isize + i;
                    let y = cell_y as isize + j;
                    if x < 0 || y < 0 {
                        continue;
                    }

                    if let Some(world_pos) = check_map(global_map, x as usize, y as usize) {
                        let dist_sq =
                            (world_pos.0 - start[0]).powi(2) + (world_pos.1 - start[1]).powi(2);
                        candidates.push((world_pos, dist_sq));
                    }
                }
            }
        }

        if !candidates.is_empty() {
            candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
            return Some(candidates[0].0);
        }

        layer += 1;
    }
    None
}
