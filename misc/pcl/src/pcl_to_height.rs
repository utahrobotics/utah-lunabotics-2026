use std::fmt::Debug;

use cubecl::{CubeType, post_processing::unroll, prelude::*, server::Binding};
use nalgebra::Vector3;

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
        })
    }
    pub fn buffer_len(&self) -> u32 {
        let size = ((self.width() * self.height()) / self.cell_size).to_f32();
        size.map_or(0, |float| float as u32)
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
        if point_height > current_height && point_height != F::new(0.0) {
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
pub fn launch_pcl_to_height<R: Runtime>(
    max_x: f32,
    min_x: f32,
    max_y: f32,
    min_y: f32,
    cell_size: f32,
    cube_size: u32,
    pcl: &Vec<Vector3<f32>>,
    device: &R::Device,
) -> Result<Vec<f32>, String> {
    let client = R::client(device);
    let pcl_shape = vec![pcl.len(), 3];
    let pcl_strides = vec![3, 1];
    let pcl = pcl
        .iter()
        .flat_map(|v| v.iter())
        .cloned()
        .collect::<Vec<f32>>();
    let pcl_handle = client.create(f32::as_bytes(&pcl));

    let bounds = MapBounds::new(max_x, min_x, max_y, min_y, cell_size)?;
    let hmap_handle = client.empty((bounds.buffer_len() * size_of::<f32>() as u32) as usize);
    unsafe {
        pcl_to_height::launch_unchecked(
            &client,
            CubeCount::Dynamic(Binding::new(
                pcl_handle.clone().binding().memory,
                pcl_handle.offset_start,
                pcl_handle.offset_end,
                pcl_handle.stream,
                0,
                ((size_of::<f32>() * 3) * pcl.len()) as u64,
            )),
            CubeDim::new_2d(cube_size, cube_size),
            MapBoundsLaunch::<f32, R>::new(
                ScalarArg::new(max_x),
                ScalarArg::new(min_x),
                ScalarArg::new(max_y),
                ScalarArg::new(min_x),
                ScalarArg::new(cell_size),
            ),
            TensorArg::from_raw_parts::<f32>(&pcl_handle, &pcl_strides, &pcl_shape, 1),
            ArrayArg::from_raw_parts_and_size(
                &hmap_handle,
                bounds.buffer_len() as usize,
                1,
                size_of::<f32>(),
            ),
        );
    }

    let hmap = client.read_one(hmap_handle);
    let output = f32::from_bytes(&hmap);
    Ok(output.to_vec())
}
