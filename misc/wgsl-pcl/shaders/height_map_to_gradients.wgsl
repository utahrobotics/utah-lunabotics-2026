@group(0) @binding(0) var<uniform> map_width: u32;
@group(0) @binding(1) var<uniform> map_height: u32;

@group(1) @binding(0) var<storage, read_write> height_map: array<f32>;
@group(1) @binding(1) var<storage, read_write> gradient_map: array<f32>;

override WORKGROUP_X: u32 = 8;
override WORKGROUP_Y: u32 = 8;

override KERNEL_RADIUS: u32 = 1;

override CELL_SIZE: f32 = 1.0; 

const UNKNOWN_VALUE: f32 = -3.40282347e+38; 

fn xy_to_index(x: i32, y: i32) -> i32 {
    if x < 0 || y < 0 || x >= i32(map_width) || y >= i32(map_height) {
        return -1;
    }
    return x + y * i32(map_width);
}

@compute
@workgroup_size(WORKGROUP_X, WORKGROUP_Y, 1)
fn gradient(
    @builtin(global_invocation_id) global_invocation_id : vec3u
) {
    let x = i32(global_invocation_id.x);
    let y = i32(global_invocation_id.y);
    
    if u32(x) >= map_width || u32(y) >= map_height {
        return;
    }
    
    let center_index = xy_to_index(x, y);

    let center_value = height_map[center_index];
    
    if center_value == UNKNOWN_VALUE {
        gradient_map[center_index] = UNKNOWN_VALUE;
        return;
    }
    
    var grad_x_sum = 0.0;
    var grad_y_sum = 0.0;
    var total_weight_x = 0.0;
    var total_weight_y = 0.0;
    
    let radius = i32(KERNEL_RADIUS);
    let run_distance = f32(radius * 2) * CELL_SIZE;
    
    for (var dy = -radius; dy <= radius; dy++) {
        let left_index = xy_to_index(x - radius, y + dy);
        let right_index = xy_to_index(x + radius, y + dy);
        
        if left_index >= 0 && right_index >= 0 {
            let left_val = height_map[u32(left_index)];
            let right_val = height_map[u32(right_index)];
            
            if left_val != UNKNOWN_VALUE && right_val != UNKNOWN_VALUE {
                let weight = 1.0 / (1.0 + f32(abs(dy)));
                
                grad_x_sum += (right_val - left_val) * weight;
                total_weight_x += weight;
            }
        }
    }
    
    for (var dx = -radius; dx <= radius; dx++) {
        let top_index = xy_to_index(x + dx, y - radius);
        let bottom_index = xy_to_index(x + dx, y + radius);
        
        if top_index >= 0 && bottom_index >= 0 {
            let top_val = height_map[u32(top_index)];
            let bottom_val = height_map[u32(bottom_index)];
            
            if top_val != UNKNOWN_VALUE && bottom_val != UNKNOWN_VALUE {
                let weight = 1.0 / (1.0 + f32(abs(dx)));
                
                grad_y_sum += (bottom_val - top_val) * weight;
                total_weight_y += weight;
            }
        }
    }
    
    var final_slope_x = 0.0;
    var final_slope_y = 0.0;
    
    if total_weight_x > 0.0 {
        final_slope_x = (grad_x_sum / total_weight_x) / run_distance;
    }

    if total_weight_y > 0.0 {
        final_slope_y = (grad_y_sum / total_weight_y) / run_distance;
    }

    if total_weight_x > 0.0 || total_weight_y > 0.0 {
        let gradient_magnitude = sqrt(final_slope_x * final_slope_x + final_slope_y * final_slope_y);
        gradient_map[center_index] = gradient_magnitude;
    } else {
        gradient_map[center_index] = UNKNOWN_VALUE;
    }
}