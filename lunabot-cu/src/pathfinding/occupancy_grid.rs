use cu_bincode::Encode;
use rerun::Color;
use rerun::Points2D;
use serde::Deserialize;
use serde::Serialize;
use std::fmt::Debug;
use std::io;
use wgsl_pcl::map_layout::MapLayout;

use crate::rerun_viz::RECORDER;

#[derive(Serialize, Encode, cu_bincode::Decode, Clone, Debug, Deserialize)]
pub struct OccupancyGrid {
    /// the layout describes the size and resolution of the map
    /// the layout is not sufficient to interpret the map data alone, the origin field is also needed
    /// however the origin is automatically applied in some getter methods such as gradient_closest_to and world_to_cell
    pub layout: MapLayout,
    pub gradient_map: Vec<f32>,
    pub origin: (f32, f32),
}

impl OccupancyGrid {
    /// Shamelessly stolen from nav2 lmao:
    /// Expands obstacles using a BFS from all obstacle cells outward.
    ///
    /// Any cell with a gradient >= `obstacle_threshold` is considered an obstacle source.
    /// All cells within `robot_radius` (in meters) of an obstacle source will have their gradient
    /// set to at least the source obstacle's gradient value (max wins if multiple sources overlap).
    ///
    /// This uses distance-binned BFS so each cell is visited at most once by the nearest obstacle,
    /// avoiding cascading inflation artifacts that a naive in-place approach would cause.
    pub fn expand_obstacles(&self, robot_radius: f32, obstacle_threshold: f32) -> Option<Self> {
        let cells_x = self.cells_x();
        let cells_y = self.cells_y();
        let cell_radius_f = robot_radius / self.layout.cell_size;
        let cell_radius = cell_radius_f.ceil() as isize;

        if cell_radius == 0 || cells_x == 0 || cells_y == 0 {
            return None;
        }

        // --- Precompute integer distance bins ---
        // Sort all (dx, dy) offsets within the radius by squared distance,
        // then assign incrementing integer levels to each unique squared distance.
        // This lets us use a Vec<Vec<CellData>> as a bucket queue instead of a heap.
        let r = cell_radius;
        let size = (r * 2 + 1) as usize;

        let mut points: Vec<(isize, isize)> = Vec::new();
        for dy in -r..=r {
            for dx in -r..=r {
                if dx * dx + dy * dy <= r * r {
                    points.push((dx, dy));
                }
            }
        }
        points.sort_by_key(|&(dx, dy)| dx * dx + dy * dy);

        let mut distance_matrix = vec![vec![0u32; size]; size];
        let mut level = 0u32;
        let mut last_dist_sq = -1i64;
        for &(dx, dy) in &points {
            let dist_sq = (dx * dx + dy * dy) as i64;
            if dist_sq != last_dist_sq {
                level += 1;
                last_dist_sq = dist_sq;
            }
            distance_matrix[(dx + r) as usize][(dy + r) as usize] = level;
        }
        let max_level = level as usize;

        // --- Seed obstacle cells intomut bin 0 ---
        // (mx, my, src_x, src_y) - current cell and the obstacle source it came from
        let mut inflation_bins: Vec<Vec<(usize, usize, usize, usize)>> =
            vec![Vec::new(); max_level + 1];

        for cy in 0..cells_y {
            for cx in 0..cells_x {
                let idx = cx + cy * cells_x;
                let val = self.gradient_map[idx];
                if val != f32::MIN && val >= obstacle_threshold {
                    inflation_bins[0].push((cx, cy, cx, cy));
                }
            }
        }

        // --- BFS by increasing distance ---
        let mut seen = vec![false; cells_x * cells_y];
        let mut output = self.gradient_map.clone();

        // just for the logger
        let mut new_obstacles = Vec::new();

        for dist in 0..=max_level {
            // Take the bin out to avoid borrow conflicts when pushing to other bins.
            // This is safe because cardinal neighbors always land in a strictly higher distance bin.
            let bin = std::mem::take(&mut inflation_bins[dist]);

            for &(mx, my, sx, sy) in &bin {
                let index = mx + my * cells_x;

                if seen[index] {
                    continue;
                }
                seen[index] = true;

                // Propagate source obstacle's gradient (max wins)
                let src_gradient = self.gradient_map[sx + sy * cells_x];
                if output[index] == f32::MIN || output[index] < src_gradient {
                    output[index] = src_gradient;
                    new_obstacles.push(index);
                }

                // Enqueue 4-connected neighbors
                const DIRS: [(isize, isize); 4] = [(-1, 0), (1, 0), (0, -1), (0, 1)];
                for (ddx, ddy) in DIRS {
                    let nx = mx as isize + ddx;
                    let ny = my as isize + ddy;
                    if nx < 0 || ny < 0 || nx >= cells_x as isize || ny >= cells_y as isize {
                        continue;
                    }
                    let (nx, ny) = (nx as usize, ny as usize);
                    let n_index = nx + ny * cells_x;
                    if seen[n_index] {
                        continue;
                    }

                    // Distance from this neighbor back to the original obstacle source
                    let dx = nx as isize - sx as isize;
                    let dy = ny as isize - sy as isize;
                    if dx.abs() > r || dy.abs() > r {
                        continue;
                    }

                    let dist_to_src = ((dx * dx + dy * dy) as f32).sqrt();
                    if dist_to_src > cell_radius_f {
                        continue;
                    }

                    let bin_idx = distance_matrix[(dx + r) as usize][(dy + r) as usize] as usize;
                    inflation_bins[bin_idx].push((nx, ny, sx, sy));
                }
            }
        }
        let expanded = OccupancyGrid {
            gradient_map: output,
            layout: self.layout.clone(),
            origin: self.origin,
        };

        if let Some(logger) = RECORDER.get() {
            let _ = logger.recorder.log(
                "ai/expanded_obstacles",
                &Points2D::new(
                    new_obstacles
                        .iter()
                        .map(|index| expanded.linear_cell_to_world(*index))
                        .flatten(),
                )
                .with_colors((0..new_obstacles.len()).map(|_| Color::from_rgb(100, 0, 100))),
            );
        }

        Some(expanded)
    }

    pub fn set_gradient_at(
        &mut self,
        cell_x: usize,
        cell_y: usize,
        value: f32,
    ) -> Result<(), String> {
        let cells_x = self.cells_x();
        let cells_y = self.cells_y();
        if cell_x >= cells_x || cell_y >= cells_y {
            return Err("Cell coordinates out of bounds".to_string());
        }
        let index = cell_x + cell_y * cells_x;
        if index >= self.gradient_map.len() {
            return Err("Index out of bounds".to_string());
        }
        self.gradient_map[index] = value;
        Ok(())
    }

    pub fn cells_x(&self) -> usize {
        ((self.layout.max_x - self.layout.min_x) / self.layout.cell_size).ceil() as usize
    }

    pub fn cells_y(&self) -> usize {
        ((self.layout.max_y - self.layout.min_y) / self.layout.cell_size).ceil() as usize
    }

    /// Get gradient value at cell coordinates
    /// Returns None if a cell has not yet been mapped
    /// Returns Err if a cell is out of bounds
    pub fn gradient_at(&self, cell_x: usize, cell_y: usize) -> Result<Option<f32>, std::io::Error> {
        let cells_x = self.cells_x();
        let cells_y = self.cells_y();
        if cell_x >= cells_x || cell_y >= cells_y {
            return Err(io::Error::other("cell out of bounds"));
        }
        let index = cell_x + cell_y * cells_x;

        Ok(self
            .gradient_map
            .get(index)
            .copied()
            .filter(|&val| val != f32::MIN))
    }

    /// Get gradient value at world coordinates
    /// uses origin to convert world coordinates to map-local coordinates
    /// returns Err if a cell is out of bounds, None if it is unknown
    pub fn gradient_closest_to(&self, x: f32, y: f32) -> Result<Option<f32>, std::io::Error> {
        let (cell_x, cell_y) = self.world_to_cell(x, y)?;
        self.gradient_at(cell_x, cell_y)
    }

    /// returns average gradient around cell
    /// returns Err if central cell is out of bounds
    /// returns Ok(none) if none of the cells in the kernel have been mapped
    pub fn gradient_around_cell(
        &self,
        cell_x: usize,
        cell_y: usize,
        kernel_size: usize,
    ) -> Result<Option<f32>, io::Error> {
        if self.gradient_at(cell_x, cell_y)?.is_none() {
            return Ok(None);
        }
        let mut gradients = Vec::new();
        let half_kernel = kernel_size as isize / 2;
        for i in -half_kernel..=half_kernel {
            for j in -half_kernel..=half_kernel {
                let nx = cell_x as isize + i;
                let ny = cell_y as isize + j;
                if nx < 0 || ny < 0 {
                    continue;
                }
                if let Ok(Some(grad)) = self.gradient_at(nx as usize, ny as usize) {
                    gradients.push(grad);
                }
            }
        }
        if gradients.is_empty() {
            Ok(None)
        } else {
            Ok(Some(gradients.iter().sum::<f32>() / gradients.len() as f32))
        }
    }

    /// returns average gradient around world coordinate
    /// returns none if no cells in the kernel have been mapped yet
    pub fn gradient_around(
        &self,
        x: f32,
        y: f32,
        kernel_size: usize,
    ) -> Result<Option<f32>, io::Error> {
        let (cell_x, cell_y) = self.world_to_cell(x, y)?;
        if self.gradient_at(cell_x, cell_y)?.is_none() {
            return Ok(None);
        }
        let mut gradients = Vec::new();
        let half_kernel = kernel_size as isize / 2;
        for i in -half_kernel..=half_kernel {
            for j in -half_kernel..=half_kernel {
                let nx = cell_x as isize + i;
                let ny = cell_y as isize + j;
                if nx < 0 || ny < 0 {
                    continue;
                }
                if let Ok(Some(grad)) = self.gradient_at(nx as usize, ny as usize) {
                    gradients.push(grad);
                }
            }
        }

        if gradients.is_empty() {
            return Ok(None);
        } else {
            return Ok(Some(gradients.iter().sum::<f32>() / gradients.len() as f32));
        }
    }

    /// Convert world coordinates to cell indices
    /// cell 0,0 is at (min_x, min_y)
    /// The origin offset is applied to transform world coordinates into the map's local coordinate system
    /// Returns Err if cell out of bounds
    pub fn world_to_cell(&self, x: f32, y: f32) -> Result<(usize, usize), std::io::Error> {
        // Convert world coordinates to map-local coordinates by subtracting the origin
        let local_x = x - self.origin.0;
        let local_y = y - self.origin.1;

        if local_x < self.layout.min_x
            || local_x >= self.layout.max_x
            || local_y < self.layout.min_y
            || local_y >= self.layout.max_y
        {
            return Err(io::Error::other("cell out of bounds"));
        }
        let cell_x = ((local_x - self.layout.min_x) / self.layout.cell_size).floor() as usize;
        let cell_y = ((local_y - self.layout.min_y) / self.layout.cell_size).floor() as usize;
        Ok((cell_x, cell_y))
    }

    /// Convert cell indices to world coordinates (returns cell center)
    /// The origin offset is applied to transform map-local coordinates into world coordinates
    /// returns Err if cell out of bounds
    pub fn cell_to_world(&self, cell_x: usize, cell_y: usize) -> Result<(f32, f32), io::Error> {
        let cells_x = self.cells_x();
        let cells_y = self.cells_y();
        if cell_x >= cells_x || cell_y >= cells_y {
            return Err(io::Error::other("cell out of bounds"));
        }
        let local_x = self.layout.min_x + (cell_x as f32 + 0.5) * self.layout.cell_size;
        let local_y = self.layout.min_y + (cell_y as f32 + 0.5) * self.layout.cell_size;
        Ok((local_x + self.origin.0, local_y + self.origin.1))
    }

    pub fn linear_cell_to_world(&self, cell_linear_index: usize) -> Result<(f32, f32), io::Error> {
        let y = cell_linear_index / self.cells_x();
        let x = cell_linear_index % self.cells_x();
        self.cell_to_world(x, y)
    }

    /// registers self into another map, overwriting any affected cells
    pub fn append_to(
        &self,
        global_map: &mut OccupancyGrid,
    ) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        for x in 0..self.cells_x() {
            for y in 0..self.cells_y() {
                // cell_to_world now returns world coordinates (already includes local map's origin)
                let Ok(world_coords) = self.cell_to_world(x, y) else {
                    continue;
                };
                let Ok(Some(gradient)) = self.gradient_at(x, y) else {
                    continue;
                };

                if !global_map
                    .layout
                    .is_in_bounds(world_coords.0, world_coords.1)
                {
                    continue;
                }
                let Ok((gx, gy)) = global_map.world_to_cell(world_coords.0, world_coords.1) else {
                    continue;
                };
                global_map.set_gradient_at(gx, gy, gradient)?;
            }
        }

        Ok(())
    }
}

impl Default for OccupancyGrid {
    fn default() -> Self {
        OccupancyGrid {
            layout: MapLayout::new(0.0, 0.0, 0.0, 0.0, 0.1),
            gradient_map: Vec::new(),
            origin: (0.0, 0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_grid(cells_x: usize, cells_y: usize, cell_size: f32) -> OccupancyGrid {
        let max_x = cells_x as f32 * cell_size;
        let max_y = cells_y as f32 * cell_size;
        OccupancyGrid {
            layout: MapLayout::new(max_x, 0.0, max_y, 0.0, cell_size),
            gradient_map: vec![f32::MIN; cells_x * cells_y],
            origin: (0.0, 0.0),
        }
    }

    #[test]
    fn single_obstacle_expands_circularly() {
        let mut grid = make_grid(20, 20, 0.1); // 2m x 2m
        // Place one obstacle at center (10, 10)
        grid.gradient_map[10 + 10 * 20] = 1.0;

        grid.expand_obstacles(0.3, 0.5); // 3-cell radius

        // The obstacle source should still be 1.0
        assert_eq!(grid.gradient_map[10 + 10 * 20], 1.0);

        // Cells within radius should be inflated
        assert_eq!(grid.gradient_map[11 + 10 * 20], 1.0); // 1 cell away
        assert_eq!(grid.gradient_map[10 + 12 * 20], 1.0); // 2 cells away
        assert_eq!(grid.gradient_map[12 + 12 * 20], 1.0); // sqrt(8)*0.1 = 0.28m < 0.3m

        // Cells outside radius should be untouched
        assert_eq!(grid.gradient_map[10 + 14 * 20], f32::MIN); // 4 cells = 0.4m > 0.3m
    }

    #[test]
    fn below_threshold_not_expanded() {
        let mut grid = make_grid(10, 10, 0.1);
        grid.gradient_map[5 + 5 * 10] = 0.1; // below threshold

        grid.expand_obstacles(0.3, 0.5);

        // Neighbors should remain unmapped
        assert_eq!(grid.gradient_map[6 + 5 * 10], f32::MIN);
    }

    #[test]
    fn max_gradient_wins_overlap() {
        let mut grid = make_grid(20, 20, 0.1);
        grid.gradient_map[5 + 10 * 20] = 0.8;
        grid.gradient_map[9 + 10 * 20] = 1.5;

        grid.expand_obstacles(0.3, 0.5);

        // Cell at (7, 10) is within radius of both obstacles
        // The higher gradient (1.5) should win
        let val = grid.gradient_map[7 + 10 * 20];
        assert!(val >= 1.5, "expected >= 1.5, got {}", val);
    }

    #[test]
    fn unmapped_cells_not_treated_as_obstacles() {
        let grid = make_grid(10, 10, 0.1);
        // All cells are f32::MIN (unmapped), none should be expanded
        grid.expand_obstacles(0.3, 0.5);

        for &val in &grid.gradient_map {
            assert_eq!(val, f32::MIN);
        }
    }
}
