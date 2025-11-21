use std::fmt::Debug;

use cubecl::{CubeType, post_processing::unroll, prelude::*};

#[derive(CubeType, CubeLaunch, Clone, Copy, Debug)]
pub struct MapBounds<F: Float> {
    /// X is refered to as width of the height map
    /// Units: Meters
    pub max_x: F,
    pub min_x: F,
    /// Y is referred to as height of the heightmap
    /// Units: Meters
    pub max_y: F,
    pub min_y: F,

    /// Size of each cell in the heightmap in meters
    pub cell_size: F,

    /// private field to force people to use the constructor for bounds checking
    _force_constructor: (),
}

impl<F: Float> MapBounds<F> {
    pub fn new(max_x: F, min_x: F, max_y: F, min_y: F, cell_size: F) -> Result<Self, String> {
        if max_x <= min_x {
            return Err("max_x must be greater than min_x".to_string());
        }
        if max_y <= min_y {
            return Err("max_y must be greater than min_y".to_string());
        }

        Ok(Self {
            max_x,
            min_x,
            max_y,
            min_y,
            cell_size,
            _force_constructor: (),
        })
    }
}

#[cube]
impl<F: Float> MapBounds<F> {
    pub fn width(&self) -> F {
        self.max_x - self.min_x
    }

    pub fn height(&self) -> F {
        self.max_y - self.min_y
    }

    pub fn within_bounds(&self, x: F, y: F) -> bool {
        let fx_max_x = self.max_x;
        let fx_min_x = self.min_x;
        let fx_max_y = self.max_y;
        let fx_min_y = self.min_y;

        x >= fx_min_x && x <= fx_max_x && y >= fx_min_y && y <= fx_max_y
    }
}

#[cube(launch_unchecked)]
fn pcl_to_height<F: Float>(
    bounds: &MapBounds<F>,
    input_pcl: &mut Tensor<F>,
    height_map: &mut Array<Atomic<F>>,
) {
    let mut point: Line<F> = Line::empty(3u32);
    for i in 0u32..3u32 {
        point[i] = input_pcl[ABSOLUTE_POS + i];
    }
    let hmap_index = point_to_heightmap_index::<F, i32>(&point, bounds);
    if hmap_index != -1 {
        let current_height = Atomic::load(&height_map[hmap_index as u32]);
        let point_height = point[2];
        if point_height > current_height {
            Atomic::store(&mut height_map[hmap_index as u32], point_height);
        }
    }
}

#[cube]
/// X axis is width
/// Y axis is height
/// returns -1 if out of bounds
fn point_to_heightmap_index<F: Float, I: Int>(point: &Line<F>, bounds: &MapBounds<F>) -> I {
    let x = point[0];
    let y = point[1];

    let min_x = bounds.min_x;
    let min_y = bounds.min_y;
    let cell_size = bounds.cell_size;

    if bounds.within_bounds(x, y) == false {
        I::cast_from(-1)
    } else {
        let width = bounds.width();
        let height = bounds.height();

        let x_index = F::floor((x - min_x) / cell_size);
        let y_index = F::floor((y - min_y) / cell_size);

        let width_cells = F::ceil(width / cell_size);

        let linear_index = y_index * width_cells + x_index;
        I::cast_from(linear_index)
    }
}

/// height_px: height of depth img in pixels
/// width_px: width of depth img in pixels
pub fn launch_pcl_to_height(
    max_x: f32,
    min_x: f32,
    max_y: f32,
    min_y: f32,
    cell_size: f32,
    height_px: f32,
    width_px: f32,
) {
    let pcl_shape = vec![height_px as usize * width_px as usize, 3];
    let pcl_strides = vec![3, 1];
}
