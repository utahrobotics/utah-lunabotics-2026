// Converts a height map to a gradient magnitude map
// if the value in the height map is f32 min, then we will know that the cell is considered to be unknown.
@group(0) @binding(0) var<storage, read_write> height_map: array<f32>;
@group(0) @binding(1) var<storage, read_write> gradient_map: array<f32>;

override MAP_WIDTH: u32;
override MAP_HEIGHT: u32;
override WORKGROUP_X: u32 = 8;
override WORKGROUP_Y: u32 = 8;
// kernel radius for gradient calculation (typically 1-3)
override KERNEL_RADIUS: u32 = 1;

@compute
@workgroup_size(WORKGROUP_X, WORKGROUP_Y, 1)
fn gradient(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
    @builtin(local_invocation_id) local_id: vec3u
) {
    let x = global_invocation_id.x;
    let y = global_invocation_id.y;
    
    if x >= MAP_WIDTH || y >= MAP_HEIGHT {
        return;
    }
    
    let center_index = xy_to_index(i32(x), i32(y));
    if center_index < 0 {
        return;
    }
    
    let center_value = height_map[center_index];
    
    if center_value == -3.40282347e+38 {
        gradient_map[center_index] = -3.40282347e+38;
        return;
    }
    
    var grad_x = 0.0;
    var grad_y = 0.0;
    var valid_samples = 0;
    
    let radius = i32(KERNEL_RADIUS);

    // sobel operator based
    
    for (var dy = -radius; dy <= radius; dy++) {
        let left_index = xy_to_index(i32(x) - radius, i32(y) + dy);
        let right_index = xy_to_index(i32(x) + radius, i32(y) + dy);
        
        if left_index >= 0 && right_index >= 0 {
            let left_val = height_map[left_index];
            let right_val = height_map[right_index];
            
            if left_val != -3.40282347e+38 && right_val != -3.40282347e+38 {
                let weight = 1.0 / (1.0 + f32(abs(dy)));
                grad_x += (right_val - left_val) * weight;
                valid_samples += 1;
            }
        }
    }
    
    for (var dx = -radius; dx <= radius; dx++) {
        let top_index = xy_to_index(i32(x) + dx, i32(y) - radius);
        let bottom_index = xy_to_index(i32(x) + dx, i32(y) + radius);
        
        if top_index >= 0 && bottom_index >= 0 {
            let top_val = height_map[top_index];
            let bottom_val = height_map[bottom_index];
            
            if top_val != -3.40282347e+38 && bottom_val != -3.40282347e+38 {
                let weight = 1.0 / (1.0 + f32(abs(dx)));
                grad_y += (bottom_val - top_val) * weight;
                valid_samples += 1;
            }
        }
    }
    
    if valid_samples > 0 {
        let gradient_magnitude = sqrt(grad_x * grad_x + grad_y * grad_y);
        gradient_map[center_index] = gradient_magnitude;
    } else {
        gradient_map[center_index] = -3.40282347e+38;
    }
}

// will return -1 if out of bounds
fn xy_to_index(x: i32, y: i32) -> i32 {
    if x < 0 || y < 0 || x >= i32(MAP_WIDTH) || y >= i32(MAP_HEIGHT) {
        return -1;
    }
    return x + y * i32(MAP_WIDTH);
}