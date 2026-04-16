use nalgebra::Vector2;

use crate::pathfinding::OccupancyGrid;

/// half_width is x 
/// half_heigh is y
/// returns basically the closest cell to the center it can find.
pub fn find_goal_in(
    occupancy_grid: &OccupancyGrid,
    center: Vector2<f32>,
    half_width: f32,
    half_height: f32,
    max_acceptable_gradient: f32,
) -> Result<Vector2<f32>, std::io::Error> {
    let (cx, cy) = occupancy_grid.world_to_cell(center.x, center.y)?;
    let (cx, cy) = (cx as i32, cy as i32);
    let hwidth_cells = (half_width / occupancy_grid.layout.cell_size) as i32;
    let hheight_cells = (half_height / occupancy_grid.layout.cell_size) as i32;

    for y_delta in 0..hheight_cells {
        for x_delta in 0..hwidth_cells {
            // (+x, +y)
            if cx + x_delta > 0 && cy + y_delta > 0 {
                if occupancy_grid.gradient_at((cx + x_delta) as usize, (cy + y_delta) as usize).is_ok_and(|inner| {
                    inner.is_none_or(|gradient| {
                        gradient < max_acceptable_gradient
                    })
                }) {
                    let (x, y) = occupancy_grid.cell_to_world((cx + x_delta) as usize, (cy + y_delta) as usize)?;
                    return Ok(Vector2::new(x, y));
                }
            }
            // (+x, -y)
            if cx + x_delta > 0 && cy - y_delta > 0 {
                if occupancy_grid.gradient_at((cx + x_delta) as usize, (cy - y_delta) as usize).is_ok_and(|inner| {
                    inner.is_none_or(|gradient| {
                        gradient < max_acceptable_gradient
                    })
                }) {
                    let (x, y) = occupancy_grid.cell_to_world((cx + x_delta) as usize, (cy - y_delta) as usize)?;
                    return Ok(Vector2::new(x, y));
                }
            }
            // (-x, +y)
            if cx - x_delta > 0 && cy + y_delta > 0 {
                if occupancy_grid.gradient_at((cx - x_delta) as usize, (cy + y_delta) as usize).is_ok_and(|inner| {
                    inner.is_none_or(|gradient| {
                        gradient < max_acceptable_gradient
                    })
                }) {
                    let (x, y) = occupancy_grid.cell_to_world((cx - x_delta) as usize, (cy + y_delta) as usize)?;
                    return Ok(Vector2::new(x, y));
                }
            }
            // (-x, -y)
            if cx - x_delta > 0 && cy - y_delta > 0 {
                if occupancy_grid.gradient_at((cx - x_delta) as usize, (cy - y_delta) as usize).is_ok_and(|inner| {
                    inner.is_none_or(|gradient| {
                        gradient < max_acceptable_gradient
                    })
                }) {
                    let (x, y) = occupancy_grid.cell_to_world((cx - x_delta) as usize, (cy - y_delta) as usize)?;
                    return Ok(Vector2::new(x, y));
                }
            }
        }
    }

    Err(std::io::Error::new(
        std::io::ErrorKind::NotFound,
        "no reachable goal found in search area",
    ))
}