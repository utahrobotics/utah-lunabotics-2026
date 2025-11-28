@group(0) @binding(0) var<storage, read> depths: array<u32>;
@group(0) @binding(1) var<storage, read_write> points: array<vec4f>;
@group(0) @binding(2) var<storage, read_write> height_map: array<atomic<u32>>;

@group(1) @binding(0) var<uniform> transform: mat4x4f;
@group(1) @binding(1) var<uniform> depth_scale: f32;
@group(1) @binding(2) var<uniform> map_layout: MapLayout;

var<workgroup> shared_transform: mat4x4f;
var<workgroup> shared_depth_scale: f32;
var<workgroup> shared_map_layout: MapLayout;
// width corresponds with x axis
var<workgroup> heightmap_width_cells: u32;
// height corresponds with y axis
var<workgroup> heightmap_height_cells: u32;

override IMAGE_WIDTH: u32 = 640;
override IMAGE_HEIGHT: u32 = 480;
override FOCAL_LENGTH_PX: f32 = 12.0;
override PRINCIPAL_POINT_PX_X: f32 = 12.0;
override PRINCIPAL_POINT_PX_Y: f32 = 12.0;
override MAX_DEPTH: f32 = 2.0;

override WORKGROUP_X: u32 = 8;
override WORKGROUP_Y: u32 = 8;
override GAUSSIAN_KERNEL_SIZE: u32 = 8;

@compute
@workgroup_size(WORKGROUP_X, WORKGROUP_Y, 1)
fn depth(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
    @builtin(local_invocation_id) local_id: vec3u
) {
    // load transform, depth scale, and map layout just once per block
    if local_id.x == 0u && local_id.y == 0u {
        shared_transform = transform;
        shared_depth_scale = depth_scale;
        shared_map_layout = map_layout;
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
        points[i].w = 0.0;
        return;
    }
    
    let x = f32(x_coord) - PRINCIPAL_POINT_PX_X;
    let y = f32(y_coord) - PRINCIPAL_POINT_PX_Y;
    let depth = f32(depthu) * shared_depth_scale;
    
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
    var point_transformed = shared_transform * vec4<f32>(transformed_point, 1.0);
    point_transformed.w = 1.0;
    points[i] = point_transformed;

    // ---------------
    // HEIGHT MAPPING
    // ---------------
    let cell_idx = point_to_cell_index(
        point_transformed.x, 
        point_transformed.y, 
        shared_map_layout
    );
    
    // Update height map if point is within bounds
    if cell_idx >= 0 {        
        if point_transformed.z > bitcast<f32>(height_map[cell_idx]) {
            let height_u32 = bitcast<u32>(point_transformed.z);            
            height_map[cell_idx] = height_u32;        
        }   
    }
}

// will return -1 if point is out of bounds
// Converts world coordinates to a cell index in the height map buffer
fn point_to_cell_index(x: f32, y: f32, map_layout: MapLayout) -> i32 {
    if x < map_layout.min_x || x > map_layout.max_x || 
       y < map_layout.min_y || y > map_layout.max_y {
        return -1;
    }
    
    let cell_x = u32(floor((x - map_layout.min_x) / map_layout.cell_size));
    let cell_y = u32(floor((y - map_layout.min_y) / map_layout.cell_size));
    
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
    // wgsl SHOULD automatically add padding here to align to 16 bytes
}