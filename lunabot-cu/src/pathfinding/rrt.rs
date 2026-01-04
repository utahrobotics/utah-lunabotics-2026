use rand::distr::{Distribution, Uniform};

use crate::tasks::OccupancyGrid;

pub fn find_path(
    local_map: &OccupancyGrid,
    global_map: Option<&OccupancyGrid>,
    start: [f32; 2],
    goal: [f32; 2],
    max_acceptable_gradient: f32,
    extend_length: f32,
    num_max_try: usize,
) -> Option<Vec<(f32, f32)>> {
    // tries local map first, then global map
    let get_gradient = |x: f32, y: f32| -> Option<f32> {
        if let Some(grad) = local_map.gradient_closest_to(x, y) {
            return Some(grad);
        }
        global_map.and_then(|gmap| gmap.gradient_closest_to(x, y))
    };

    // maps with non-zero origins, we need to transform the layout bounds to world space
    let (world_min_x, world_max_x, world_min_y, world_max_y) = if let Some(gmap) = global_map {
        (
            gmap.layout.min_x + gmap.origin.0,
            gmap.layout.max_x + gmap.origin.0,
            gmap.layout.min_y + gmap.origin.1,
            gmap.layout.max_y + gmap.origin.1,
        )
    } else {
        (
            local_map.layout.min_x + local_map.origin.0,
            local_map.layout.max_x + local_map.origin.0,
            local_map.layout.min_y + local_map.origin.1,
            local_map.layout.max_y + local_map.origin.1,
        )
    };

    if world_max_x <= world_min_x || world_max_y <= world_min_y {
        eprintln!("Invalid map dimensions: max_x <= min_x or max_y <= min_y");
        return None;
    }
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
        if let Some(valid_cell) = flood_fill_escape(
            local_map,
            global_map,
            start,
            max_acceptable_gradient,
            num_max_try,
        ) {
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
    let is_free = |point: &[f32]| {
        if let Some(grad) = get_gradient(point[0], point[1]) {
            grad <= max_acceptable_gradient
        } else {
            false
        }
    };
    let random_sample = || -> Vec<f32> {
        let mut rng: rand::prelude::ThreadRng = rand::rng();
        let x_range = Uniform::new(world_min_x, world_max_x).unwrap();
        let y_range = Uniform::new(world_min_y, world_max_y).unwrap();
        vec![x_range.sample(&mut rng), y_range.sample(&mut rng)]
    };
    let mut path = rrt::dual_rrt_connect(
        &start,
        &goal,
        is_free,
        random_sample,
        extend_length,
        num_max_try,
    )
    .ok()?;
    rrt::smooth_path(&mut path, is_free, extend_length, num_max_try);

    let path = path
        .iter()
        .map(|point| (point[0], point[1]))
        .collect::<Vec<(f32, f32)>>();
    if path.is_empty() {
        eprintln!("failed to calc path");
    }
    Some(path)
}

/// used for when the robot appears to be stuck in unknown or obstacle space
/// returns pretty much the nearest free space
fn flood_fill_escape(
    local_map: &OccupancyGrid,
    global_map: Option<&OccupancyGrid>,
    start: [f32; 2],
    max_acceptable_gradient: f32,
    num_max_try: usize,
) -> Option<(f32, f32)> {
    // local prioritized
    let (cell_x, cell_y) = local_map
        .world_to_cell(start[0], start[1])
        .or_else(|| global_map.and_then(|g| g.world_to_cell(start[0], start[1])))?;

    let mut layer = 1;
    loop {
        if layer > num_max_try as isize {
            break;
        }
        for i in -layer as isize..=layer as isize {
            for j in -layer as isize..=layer as isize {
                if i.abs() != layer && j.abs() != layer {
                    continue;
                }
                let x = cell_x as isize + i;
                let y = cell_y as isize + j;
                if x < 0 || y < 0 {
                    continue;
                }
                let found_x = (cell_x as isize + i) as usize;
                let found_y = (cell_y as isize + j) as usize;

                let grad = local_map
                    .gradient_around_cell(found_x, found_y, 10)
                    .or_else(|| {
                        global_map.and_then(|gmap| gmap.gradient_around_cell(found_x, found_y, 10))
                    });

                if let Some(grad) = grad {
                    if grad <= max_acceptable_gradient {
                        return local_map.cell_to_world(found_x, found_y).or_else(|| {
                            global_map.and_then(|g| g.cell_to_world(found_x, found_y))
                        });
                    }
                }
            }
        }
        layer += 1;
    }
    None
}
