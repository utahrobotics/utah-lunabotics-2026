// Uses a conditional mean filter to remove outliers on a height map
// Only replaces values that are statistical outliers, preserving detail

@group(0) @binding(0) var<storage, read_write> input_height_map: array<f32>;
@group(0) @binding(1) var<storage, read_write> output_filtered_height_map: array<f32>;
// @group(0) @binding(2) var<storage, read_write> original_height_map: array<f32>;

override MAP_WIDTH: u32;
override MAP_HEIGHT: u32;
override KERNEL_RADIUS: u32 = 2;
// How many standard deviations away before we consider it an outlier
// Higher values = more conservative filtering (2.0-3.0 is typical)
override OUTLIER_THRESHOLD: f32 = 2.5;

// Cells with > MAX_UNKNOWN_NEIGHBORS_RATIO will be removed
override MAX_UNKNOWN_NEIGHBORS_RATIO:f32 = 50.0;
const CLEAR_VALUE: u32 = 0xFF7FFFFF; // bitcast of f32::MIN


@compute
@workgroup_size(8, 8, 1)
fn depth(
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
    
    let center_value = input_height_map[center_index];
    
    if center_value == bitcast<f32>(CLEAR_VALUE) {
        output_filtered_height_map[center_index] = center_value;
        return;
    }
    
    let corner_x = i32(x) - (i32(KERNEL_RADIUS) - 1);
    let corner_y = i32(y) - (i32(KERNEL_RADIUS) - 1);
    
    var values: array<f32, 16>;
    var count = 0u;
    var sum = 0.0;
    var unknown_neighbors_count = 0u;
    
    for (var i = 0; i < (i32(KERNEL_RADIUS * 2)) - 1; i++) {
        for (var j = 0; j < (i32(KERNEL_RADIUS * 2)) - 1; j++) {
            let x_coord = corner_x + i;
            let y_coord = corner_y + j;
            let index = xy_to_index(x_coord, y_coord);
            
            if index >= 0 {
                let value = input_height_map[index];
                if value != bitcast<f32>(CLEAR_VALUE) {
                    values[count] = value;
                    sum += value;
                    count++;
                } else {
                    unknown_neighbors_count++;
                }
            }
        }
    }

    let total_neighbors = count + unknown_neighbors_count;
    if total_neighbors == 0u {
        output_filtered_height_map[center_index] = bitcast<f32>(CLEAR_VALUE);
        return;
    }
    
    let unknown_ratio = f32(unknown_neighbors_count) / f32(total_neighbors);
    if unknown_ratio > MAX_UNKNOWN_NEIGHBORS_RATIO {
        output_filtered_height_map[center_index] = bitcast<f32>(CLEAR_VALUE);
        return;
    }
    
    if count > 0u {
        let mean = sum / f32(count);
        
        var variance_sum = 0.0;
        for (var i = 0u; i < count; i++) {
            let diff = values[i] - mean;
            variance_sum += diff * diff;
        }
        let std_dev = sqrt(variance_sum / f32(count));
        
        let deviation = abs(center_value - mean);
        
        if deviation > OUTLIER_THRESHOLD * std_dev { // reject value
            output_filtered_height_map[center_index] = bitcast<f32>(CLEAR_VALUE);
        } else {
            output_filtered_height_map[center_index] = center_value;
        }
    } else {
        output_filtered_height_map[center_index] = center_value;
    }
}

fn xy_to_index(x: i32, y: i32) -> i32 {
    if x < 0 || y < 0 || x >= i32(MAP_WIDTH) || y >= i32(MAP_HEIGHT) {
        return -1;
    }
    return x + y * i32(MAP_WIDTH);
}
