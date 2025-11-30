// Expands obstacles in a gradient map based on robot radius
// Cells with gradient > max_gradient_for_obstacle are considered obstacles
// Unknown cells are marked with f32 min value (-3.40282347e+38)
@group(0) @binding(0) var<storage, read_write> gradient_map: array<f32>;
@group(0) @binding(1) var<storage, read_write> expanded_obstacles: array<f32>;
@group(0) @binding(2) var<uniform> max_gradient_for_obstacle: f32;

override MAP_WIDTH: u32;
override MAP_HEIGHT: u32;
override WORKGROUP_X: u32 = 8;
override WORKGROUP_Y: u32 = 8;
override ROBOT_RADIUS_METERS: f32 = 1.0;
override CELL_SIZE_METERS: f32;

@compute
@workgroup_size(WORKGROUP_X, WORKGROUP_Y, 1)
fn expand_obstacles(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
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
    
    let center_value = gradient_map[center_index];
    
    
    // Calculate search radius in cells
    let search_radius_cells = i32(ceil(ROBOT_RADIUS_METERS / CELL_SIZE_METERS));
    
    // Check if current cell is already an obstacle
    if center_value > max_gradient_for_obstacle {
        expanded_obstacles[center_index] = center_value;
        return;
    }
    
    // Search neighborhood for obstacles
    var is_near_obstacle = false;
    
    for (var dy = -search_radius_cells; dy <= search_radius_cells; dy++) {
        for (var dx = -search_radius_cells; dx <= search_radius_cells; dx++) {
            let neighbor_x = i32(x) + dx;
            let neighbor_y = i32(y) + dy;
            let neighbor_index = xy_to_index(neighbor_x, neighbor_y);
            
            if neighbor_index >= 0 {
                let neighbor_value = gradient_map[neighbor_index];
                
                // Skip unknown cells
                if neighbor_value == -3.40282347e+38 {
                    continue;
                }
                
                // Check if neighbor is an obstacle
                if neighbor_value > max_gradient_for_obstacle {
                    // Calculate actual distance to this obstacle cell
                    let dist = sqrt(f32(dx * dx + dy * dy)) * CELL_SIZE_METERS;
                    
                    // If within robot radius, mark as expanded obstacle
                    if dist <= ROBOT_RADIUS_METERS {
                        is_near_obstacle = true;
                        break;
                    }
                }
            }
        }
        
        if is_near_obstacle {
            break;
        }
    }
    
    // Mark cell as obstacle if near one, otherwise keep original value
    if is_near_obstacle {
        // Use a value slightly above threshold to indicate expanded obstacle
        expanded_obstacles[center_index] = max_gradient_for_obstacle + 0.1;
    } else {
        expanded_obstacles[center_index] = center_value;
    }
}

// Returns -1 if out of bounds
fn xy_to_index(x: i32, y: i32) -> i32 {
    if x < 0 || y < 0 || x >= i32(MAP_WIDTH) || y >= i32(MAP_HEIGHT) {
        return -1;
    }
    return x + y * i32(MAP_WIDTH);
}