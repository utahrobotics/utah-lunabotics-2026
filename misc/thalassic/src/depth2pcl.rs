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
const MIN_STRIDE: NonZeroU32 = {{min_stride}};
const MAX_STRIDE: NonZeroU32 = {{max_stride}};
const STRIDE_TRANSITION_START: f32 = {{stride_transition_start}};
const STRIDE_TRANSITION_END: f32 = {{stride_transition_end}};

fn calculate_stride(depth: f32) -> u32 {
    if depth <= STRIDE_TRANSITION_START {
        return MAX_STRIDE;
    } else if depth >= STRIDE_TRANSITION_END {
        return MIN_STRIDE;
    } else {
        let t = (depth - STRIDE_TRANSITION_START) / (STRIDE_TRANSITION_END - STRIDE_TRANSITION_START);
        return u32(mix(f32(MAX_STRIDE), f32(MIN_STRIDE), t));
    }
}

@compute
@workgroup_size(8, 8, 1)
fn depth(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
) {
    let output_i = global_invocation_id.x + global_invocation_id.y * (IMAGE_WIDTH / MIN_STRIDE);

    if output_i >= PIXEL_COUNT {
        return;
    }

    let base_x = global_invocation_id.x * MIN_STRIDE;
    let base_y = global_invocation_id.y * MIN_STRIDE;

    if base_x >= IMAGE_WIDTH {
        points[output_i].w = 0.0;
        return;
    }

    let base_pixel_i = base_x + base_y * IMAGE_WIDTH;
    if base_pixel_i >= PIXEL_COUNT {
        points[output_i].w = 0.0;
        return;
    }

    let double_depth = depths[base_pixel_i / 2];
    var depthu: u32;
    if base_pixel_i % 2 == 1 {
        depthu = double_depth >> 16;
    } else {
        depthu = double_depth & 0xFFFF;
    }

    if depthu == 0 {
        points[output_i].w = 0.0;
        return;
    }

    let depth = f32(depthu) * depth_scale;
    if depth > MAX_DEPTH {
        points[output_i].w = 0.0;
        return;
    }

    let ideal_stride = calculate_stride(depth);

    if (base_x % ideal_stride != 0) || (base_y % ideal_stride != 0) {
        points[output_i].w = 0.0;
        return;
    }

    let x = f32(base_x) - PRINCIPAL_POINT_PX.x;
    let y = f32(base_y) - PRINCIPAL_POINT_PX.y;

    let new_scale = depth / FOCAL_LENGTH_PX;
    var point = vec3(x, -y, 0.0) * new_scale;
    point.z = -depth;

    let transformed_point = vec3(-point.z, -point.x, point.y);
    var point_transformed = transform * vec4<f32>(transformed_point, 1.0);
    point_transformed.w = 1.0;

    points[output_i] = point_transformed;
}
"#
);
