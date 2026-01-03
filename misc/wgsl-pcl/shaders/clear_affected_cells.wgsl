@group(1) @binding(0) var<storage, read> depths: array<u32>;
@group(1) @binding(1) var<storage, read_write> height_map: array<atomic<u32>>;

@group(0) @binding(0) var<uniform> transform: mat4x4f;
@group(0) @binding(1) var<uniform> depth_scale: f32;
@group(0) @binding(2) var<uniform> map_layout: MapLayout;

var<workgroup> heightmap_width_cells: u32;
var<workgroup> heightmap_height_cells: u32;

override IMAGE_WIDTH: u32 = 640;
override IMAGE_HEIGHT: u32 = 480;
override FOCAL_LENGTH_PX: f32 = 12.0;
override PRINCIPAL_POINT_PX_X: f32 = 12.0;
override PRINCIPAL_POINT_PX_Y: f32 = 12.0;
override MAX_DEPTH: f32 = 2.0;
override MIN_DEPTH: f32 = 0.20;

override WORKGROUP_X: u32 = 8;
override WORKGROUP_Y: u32 = 8;


override MIN_DISTANCE_TO_CLEAR: f32 = 0.8;

const CLEAR_VALUE: u32 = 0xFF7FFFFF; 

@compute
@workgroup_size(WORKGROUP_X, WORKGROUP_Y, 1)
fn clear_affected(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
    @builtin(local_invocation_id) local_id: vec3u
) {
    if local_id.x == 0u && local_id.y == 0u {
        let width = map_layout.max_x - map_layout.min_x;
        let height = map_layout.max_y - map_layout.min_y;
        heightmap_width_cells = u32(ceil(width / map_layout.cell_size));
        heightmap_height_cells = u32(ceil(height / map_layout.cell_size));
    }
    workgroupBarrier();
    
    let x_coord = global_invocation_id.x;
    let y_coord = global_invocation_id.y;
    
    if x_coord >= IMAGE_WIDTH || y_coord >= IMAGE_HEIGHT {
        return;
    }
    
    let i = x_coord + y_coord * IMAGE_WIDTH;
    
    let double_depth = depths[i / 2];
    var depthu: u32;
    if i % 2 == 1 {
        depthu = double_depth >> 16;
    } else {
        depthu = double_depth & 0xFFFF;
    }
    
    if depthu == 0 {
        return;
    }
    
    let x = f32(x_coord) - PRINCIPAL_POINT_PX_X;
    let y = f32(y_coord) - PRINCIPAL_POINT_PX_Y;
    let depth = f32(depthu) * depth_scale;
    
    if depth > MAX_DEPTH || depth < MIN_DEPTH || depth < MIN_DISTANCE_TO_CLEAR {
        return;
    }
    
    let new_scale = depth / FOCAL_LENGTH_PX;
    var point = vec3(x, -y, 0.0) * new_scale;
    point.z = -depth;
    
    let point_in_camera = vec3(-point.z, -point.x, point.y);
    var point_transformed = transform * vec4<f32>(point_in_camera, 1.0);
    
    let cell_x = u32(floor((point_transformed.x - map_layout.min_x) / map_layout.cell_size));
    let cell_y = u32(floor((point_transformed.y - map_layout.min_y) / map_layout.cell_size));

    let cell_idx = cell_xy_to_linear_address(cell_x, cell_y, map_layout);
    
    if cell_idx >= 0 {
        atomicStore(&height_map[cell_idx], CLEAR_VALUE);
    }
}

fn cell_xy_to_linear_address(cell_x: u32, cell_y: u32, map_layout: MapLayout) -> i32 {
    if cell_x >= heightmap_width_cells || cell_y >= heightmap_height_cells {
        return -1;
    }
    
    return i32(cell_x + cell_y * heightmap_width_cells);
}

struct MapLayout {
    max_x: f32,
    min_x: f32,
    max_y: f32,
    min_y: f32,
    cell_size: f32,
}

