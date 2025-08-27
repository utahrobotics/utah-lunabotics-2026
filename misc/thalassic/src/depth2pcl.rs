use gputter::build_shader;
build_shader!(
pub(crate) Depth2Pcl,
r#"
#[buffer] var<storage, read> depths: array<u32, HALF_PIXEL_COUNT>;
#[buffer] var<storage, read_write> points: array<vec4f, PIXEL_COUNT>;
#[buffer] var<uniform> transform: mat4x4f;
#[buffer] var<uniform> depth_scale: f32;
const IMAGE_WIDTH: NonZeroU32 = {{image_width}};
const FOCAL_LENGTH_PX: f32 = {{focal_length_px}};
const PRINCIPAL_POINT_PX: vec2f = {{principal_point_px}};
const PIXEL_COUNT: NonZeroU32 = {{pixel_count}};
const HALF_PIXEL_COUNT: NonZeroU32 = {{half_pixel_count}};
const MAX_DEPTH: f32 = {{max_depth}};
const MIN_STRIDE: NonZeroU32 = {{min_stride}};  // Minimum stride (far from camera)
const MAX_STRIDE: NonZeroU32 = {{max_stride}};  // Maximum stride (close to camera)
const STRIDE_TRANSITION_START: f32 = {{stride_transition_start}};  // Depth where stride starts decreasing
const STRIDE_TRANSITION_END: f32 = {{stride_transition_end}};      // Depth where stride reaches minimum

fn calculate_stride(depth: f32) -> u32 {
    if depth <= STRIDE_TRANSITION_START {
        return MAX_STRIDE;
    } else if depth >= STRIDE_TRANSITION_END {
        return MIN_STRIDE;
    } else {
        // Linear interpolation between MAX_STRIDE and MIN_STRIDE
        let t = (depth - STRIDE_TRANSITION_START) / (STRIDE_TRANSITION_END - STRIDE_TRANSITION_START);
        return u32(mix(f32(MAX_STRIDE), f32(MIN_STRIDE), t));
    }
}

@compute
@workgroup_size(8, 8, 1)
fn depth(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
) {
    // Calculate output index in the grid
    let width_over_min_stride = IMAGE_WIDTH / MIN_STRIDE;
    let output_i = global_invocation_id.x + global_invocation_id.y * width_over_min_stride;

    if output_i >= PIXEL_COUNT {
        return;
    }

    // Try different strides to find the best one for this grid position
    // Start with minimum stride and work up
    var best_stride = MIN_STRIDE;
    var best_depth = 0.0f;

    // Test each possible stride
    for (var test_stride = MIN_STRIDE; test_stride <= MAX_STRIDE; test_stride++) {
        let test_x = global_invocation_id.x * test_stride;
        let test_y = global_invocation_id.y * test_stride;

        if test_x >= IMAGE_WIDTH {
            break;
        }

        let test_pixel_i = test_x + test_y * IMAGE_WIDTH;
        if test_pixel_i >= PIXEL_COUNT {
            break;
        }

        // Get the depth value at this test position
        let double_depth = depths[test_pixel_i / 2];
        var depthu: u32;
        if test_pixel_i % 2 == 1 {
            depthu = double_depth >> 16;
        } else {
            depthu = double_depth & 0xFFFF;
        }

        if depthu == 0 {
            continue; // Skip invalid depths
        }

        let test_depth = f32(depthu) * depth_scale;
        if test_depth > MAX_DEPTH {
            continue; // Skip depths that are too far
        }

        // Check if this stride is appropriate for this depth
        let ideal_stride = calculate_stride(test_depth);

        // Use the stride if it matches what we'd expect for this depth
        // Allow some tolerance to avoid being too strict
        if test_stride <= ideal_stride {
            best_stride = test_stride;
            best_depth = test_depth;
            break; // Use the first (smallest) valid stride
        }
    }

    // If no valid depth was found, mark as invalid
    if best_depth == 0.0 {
        points[output_i].w = 0.0;
        return;
    }

    // Calculate the final pixel position using the best stride
    let final_x = global_invocation_id.x * best_stride;
    let final_y = global_invocation_id.y * best_stride;
    let final_pixel_i = final_x + final_y * IMAGE_WIDTH;

    // Re-fetch the depth at the final position (should be the same as best_depth)
    let double_depth_final = depths[final_pixel_i / 2];
    var depthu_final: u32;
    if final_pixel_i % 2 == 1 {
        depthu_final = double_depth_final >> 16;
    } else {
        depthu_final = double_depth_final & 0xFFFF;
    }

    if depthu_final == 0 {
        points[output_i].w = 0.0;
        return;
    }

    let x = f32(final_x) - PRINCIPAL_POINT_PX.x;
    let y = f32(final_y) - PRINCIPAL_POINT_PX.y;
    let depth_final = f32(depthu_final) * depth_scale;

    if depth_final > MAX_DEPTH {
        points[output_i].w = 0.0;
        return;
    }

    let new_scale = depth_final / FOCAL_LENGTH_PX;
    var point = vec3(x, -y, 0.0) * new_scale;
    point.z = -depth_final;

    // Transform coordinate system: x=right, y=up, z=forward(negative)
    // to: x=forward, y=left, z=up
    let transformed_point = vec3(-point.z, -point.x, point.y);
    var point_transformed = transform * vec4<f32>(transformed_point, 1.0);
    point_transformed.w = 1.0;

    points[output_i] = point_transformed;
}
"#
);
