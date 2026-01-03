// in this case we are working on a height map, but this image could be anything
// if the value in the height map is f32 min, then we will know that the cell is considered to be unknown.
@group(0) @binding(0) var<uniform> map_width: u32;
@group(0) @binding(1) var<uniform> map_height: u32;
@group(1) @binding(0) var<storage, read_write> height_map: array<f32>;
@group(1) @binding(1) var<storage, read_write> filtered_height_map: array<f32>;

override WORKGROUP_X: u32 = 8;
override WORKGROUP_Y: u32 = 8;
// kernel radius includes the center pixel in the kernel
override KERNEL_RADIUS: u32 = 4;
// sigma controls the spread of the Gaussian
override SIGMA: f32 = 3.0;
override CELL_SIZE: f32 = 0.1;

@compute
@workgroup_size(WORKGROUP_X, WORKGROUP_Y, 1)
fn depth(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
    @builtin(local_invocation_id) local_id: vec3u
) {
    let x = global_invocation_id.x;
    let y = global_invocation_id.y;
    
    if x >= map_width || y >= map_height {
        return;
    }
    
    let center_index = xy_to_index(i32(x), i32(y));
    if center_index < 0 {
        return;
    }
    
    let center_value = height_map[center_index];
    
    // Skip unknown cells (f32 min value)
    if center_value == -3.40282347e+38 {
        filtered_height_map[center_index] = center_value;
        return;
    }
    
    let corner_x = i32(x) - (i32(KERNEL_RADIUS) - 1);
    let corner_y = i32(y) - (i32(KERNEL_RADIUS) - 1);
    
    var weighted_sum = 0.0;
    var weight_sum = 0.0;
    
    // the -1 is because the center pixel is counted in the radius
    for (var i = 0; i < (i32(KERNEL_RADIUS * 2)) - 1; i++) {
        for (var j = 0; j < (i32(KERNEL_RADIUS * 2)) - 1; j++) {
            let x_coord = corner_x + i;
            let y_coord = corner_y + j;
            let index = xy_to_index(x_coord, y_coord);
            
            if index >= 0 {
                let neighbor_value = height_map[index];
                
                if neighbor_value == -3.40282347e+38 {
                    continue;
                }
                
                let dx = f32(x_coord - i32(x)) * CELL_SIZE;
                let dy = f32(y_coord - i32(y)) * CELL_SIZE;
                let spatial_dist = sqrt(dx * dx + dy * dy);
                
                // Simple Gaussian weight based only on spatial distance
                let weight = exp(-(spatial_dist * spatial_dist) / (2.0 * SIGMA * SIGMA));
                
                weighted_sum += neighbor_value * weight;
                weight_sum += weight;
            }
        }
    }
    
    // Normalize and store result
    if weight_sum > 0.0 {
        filtered_height_map[center_index] = weighted_sum / weight_sum;
    } else {
        filtered_height_map[center_index] = center_value;
    }
}

// will return -1 if out of bounds
fn xy_to_index(x: i32, y: i32) -> i32 {
    if x < 0 || y < 0 || x >= i32(map_width) || y >= i32(map_height) {
        return -1;
    }
    return x + y * i32(map_width);
}