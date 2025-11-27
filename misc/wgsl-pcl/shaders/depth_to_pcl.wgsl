@group(0) @binding(0) var<storage, read> depths: array<u32>;
@group(0) @binding(1) var<storage, read_write> points: array<vec4f>;
@group(0) @binding(2) var<uniform> transform: mat4x4f;
@group(0) @binding(3) var<uniform> depth_scale: f32;

override IMAGE_WIDTH: u32 = 640;
override IMAGE_HEIGHT: u32 = 480;
override FOCAL_LENGTH_PX: f32 = 12.0;
override PRINCIPAL_POINT_PX_X: f32 = 12.0;
override PRINCIPAL_POINT_PX_Y: f32 = 12.0;
override MAX_DEPTH: f32 = 2.0;

@compute
@workgroup_size(8, 8, 1)
fn depth(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
) {
    let x_coord = global_invocation_id.x;
    let y_coord = global_invocation_id.y;
    
    if x_coord >= IMAGE_WIDTH || y_coord >= IMAGE_HEIGHT {
        return;
    }
    
    let i = x_coord + y_coord * IMAGE_WIDTH;
    
    // Extract 16-bit depth from packed 32-bit values
    let double_depth = depths[i / 2];
    var depthu: u32;
    if i % 2 == 1 {
        depthu = double_depth >> 16;
    } else {
        depthu = double_depth & 0xFFFF;
    }
    
    if depthu == 0 {
        points[i].w = 0.0;
        return;
    }
    
    let x = f32(x_coord) - PRINCIPAL_POINT_PX_X;
    let y = f32(y_coord) - PRINCIPAL_POINT_PX_Y;
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