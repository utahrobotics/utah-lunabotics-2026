use rand::distr::{Distribution, Uniform};

use crate::tasks::OccupancyGrid;

pub fn find_path(
    map: &OccupancyGrid,
    start: [f32; 2],
    goal: [f32; 2],
    max_acceptable_gradient: f32,
    extend_length: f32,
    num_max_try: usize,
) -> Option<Vec<(f32, f32)>> {
    if map.max_x <= map.min_x || map.max_y <= map.min_y {
        eprintln!("Invalid map dimensions: max_x <= min_x or max_y <= min_y");
        return None;
    }
    let mut initial_path = Vec::new();
    if map
        .gradient_closest_to(start[0], start[1])
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
            flood_fill_escape(map, start, max_acceptable_gradient, num_max_try)
        {
            println!("escape path found: {:?}", valid_cell);
            initial_path.push(valid_cell);
        } else {
            eprintln!("no escape path found");
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
        if let Some(grad) = map.gradient_closest_to(point[0], point[1]) {
            grad <= max_acceptable_gradient
        } else {
            false
        }
    };
    let random_sample = || -> Vec<f32> {
        let mut rng: rand::prelude::ThreadRng = rand::rng();

        // safe unwraps so long as we keep the map dimension checks above
        let x_range = Uniform::new(map.min_x, map.max_x).unwrap();
        let y_range = Uniform::new(map.min_y, map.max_y).unwrap();
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
    map: &OccupancyGrid,
    start: [f32; 2],
    max_acceptable_gradient: f32,
    num_max_try: usize,
) -> Option<(f32, f32)> {
    let (cell_x, cell_y) = map.world_to_cell(start[0], start[1])?;
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
                let grad = map.gradient_around_cell(
                    (cell_x as isize + i) as usize,
                    (cell_y as isize + j) as usize,
                    20,
                );
                if let Some(grad) = grad {
                    if grad <= max_acceptable_gradient {
                        let found_x = (cell_x as isize + i) as usize;
                        let found_y = (cell_y as isize + j) as usize;
                        return map.cell_to_world(found_x, found_y);
                    }
                }
            }
        }
        layer += 1;
    }
    None
}
