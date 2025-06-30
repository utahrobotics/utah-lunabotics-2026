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
const STRIDE: NonZeroU32 = {{stride}};

@compute
@workgroup_size(8, 8, 1)
fn depth(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
) {
    let strided_x = global_invocation_id.x * STRIDE;
    let strided_y = global_invocation_id.y * STRIDE;

    if strided_x >= IMAGE_WIDTH {
        return;
    }

    let original_i = strided_x + strided_y * IMAGE_WIDTH;
    let width_over_STRIDE = select(IMAGE_WIDTH / STRIDE, IMAGE_WIDTH, STRIDE == 0u);
    let i = global_invocation_id.x + global_invocation_id.y * width_over_STRIDE;

    if original_i >= PIXEL_COUNT {
        return;
    }

    let double_depth = depths[original_i / 2];
    var depthu: u32;
    if original_i % 2 == 1 {
        depthu = double_depth >> 16;
    } else {
        depthu = double_depth & 0xFFFF;
    }

    if depthu == 0 {
        points[i].w = 0.0;
        return;
    }

    let x = f32(strided_x) - PRINCIPAL_POINT_PX.x;
    let y = f32(strided_y) - PRINCIPAL_POINT_PX.y;
    let depth = f32(depthu) * depth_scale;

    if depth > MAX_DEPTH {
        points[i].w = 0.0;
        return;
    }

    let new_scale = depth / FOCAL_LENGTH_PX;
    var point = vec3(x, -y, 0.0) * new_scale;
    point.z = -depth;

    // Transform coordinate system: x=right, y=up, z=forward(negative) 
    // to: x=forward, y=left, z=up
    let transformed_point = vec3(-point.z, -point.x, point.y);
    
    var point_transformed = transform * vec4<f32>(transformed_point, 1.0);
    point_transformed.w = 1.0;
    points[i] = point_transformed;
}
"#
);
