use cubecl::frontend::Cast;
use cubecl::prelude::*;
use nalgebra::{Isometry3, Vector3};
pub mod utils;
pub use cubecl;
pub mod pcl_to_height;

#[cube(launch_unchecked)]
fn depth_to_pcl<F: Float>(
    input: &Tensor<u32>,
    output: &mut Tensor<F>,
    transform: &Array<F>,
    ppx: F,
    ppy: F,
    focal_length_px: F,
    max_depth: F,
    depth_scale: F,
) {
    let x = ABSOLUTE_POS_X;
    let y = ABSOLUTE_POS_Y;

    let width = input.shape(1);
    let height = input.shape(0);

    if x < width && y < height {
        let input_idx = y * input.stride(0) + x;
        let depth_raw = input[input_idx];
        let depth_m = F::cast_from(depth_raw) * depth_scale;

        let linear_index = y * width + x;

        if depth_raw != 0u32 && depth_m <= max_depth {
            let x_offset = F::cast_from(x) - ppx;
            let y_offset = F::cast_from(y) - ppy;
            let scale = depth_m / focal_length_px;

            let x_cam = x_offset * scale;
            let y_cam = F::new(0.0) - y_offset * scale;
            let z_cam = F::new(0.0) - depth_m;

            let mut point = Line::empty(4u32);
            point[0] = F::new(0.0) - z_cam;
            point[1] = F::new(0.0) - x_cam;
            point[2] = y_cam;
            point[3] = F::new(1.0);

            // Transform using matrix-vector multiplication with unrolling
            let mut result = Line::<F>::empty(3u32);
            #[unroll]
            for i in 0u32..3u32 {
                let row_base = i * 4u32;
                let mut sum = F::new(0.0);
                #[unroll]
                for j in 0u32..4u32 {
                    sum += transform[row_base + j] * point[j];
                }
                result[i] = sum;
            }

            let base_idx = linear_index * 3u32;
            output[base_idx] = result[0];
            output[base_idx + 1u32] = result[1];
            output[base_idx + 2u32] = result[2];
        } else {
            let base_idx = linear_index * 3u32;
            output[base_idx] = F::new(0.0);
            output[base_idx + 1u32] = F::new(0.0);
            output[base_idx + 2u32] = F::new(0.0);
        }
    }
}

pub fn launch_depth_to_pcl<R: Runtime>(
    depths: &[u32],
    width_px: u32,
    height_px: u32,
    ppx: f32,
    ppy: f32,
    focal_length_px: f32,
    max_depth: f32,
    depth_scale: f32,
    camera_isometry: Isometry3<f32>,
    cube_size: u32,
    device: &R::Device,
) -> Vec<Vector3<f32>> {
    let client = R::client(device);

    let transform_data: Vec<f32> = camera_isometry.to_homogeneous().as_slice().to_vec();

    let input_handle = client.create(u32::as_bytes(depths));
    let output_handle = client.empty((width_px * height_px) as usize * (size_of::<f32>() * 3));
    let transform_handle = client.create(f32::as_bytes(&transform_data));

    let input_shape = vec![height_px as usize, width_px as usize];
    let input_strides = vec![width_px as usize, 1];

    let output_shape = vec![height_px as usize * width_px as usize * 3];
    let output_strides = vec![1];

    unsafe {
        depth_to_pcl::launch_unchecked::<f32, R>(
            &client,
            CubeCount::Static(
                (width_px + cube_size - 1) / cube_size,
                (height_px + cube_size - 1) / cube_size,
                1,
            ),
            CubeDim::new(cube_size, cube_size, 1),
            TensorArg::from_raw_parts::<u32>(&input_handle, &input_strides, &input_shape, 1),
            TensorArg::<R>::from_raw_parts::<f32>(
                &output_handle,
                &output_strides,
                &output_shape,
                1,
            ),
            ArrayArg::from_raw_parts_and_size(&transform_handle, 16, 1, size_of::<f32>()),
            ScalarArg::new(ppx),
            ScalarArg::new(ppy),
            ScalarArg::new(focal_length_px),
            ScalarArg::new(max_depth),
            ScalarArg::new(depth_scale),
        );
    }

    let bytes = client.read_one(output_handle);
    let output = f32::from_bytes(&bytes);

    unsafe {
        let output: &[[f32; 3]] = &*(output as *const _ as *const [[f32; 3]]);
        return output.iter().map(|p| (*p).into()).collect();
    }
}
