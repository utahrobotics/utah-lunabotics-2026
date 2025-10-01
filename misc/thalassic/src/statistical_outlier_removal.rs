use gputter::build_shader;
build_shader!(
    pub(crate) StatisticalOutlierRemoval,
r#"
// actually an array of f32's, use bitcast<f32>
#[buffer] var<storage, read_write> height_map_in: array<u32>;
#[buffer] var<storage, read_write> filtered: array<u32>;
#[buffer] var<storage, read_write> known_cells: array<atomic<u32>, CELL_COUNT>;

const K_NEIGHBORS: u32 = {{k_neighbors}};
const STD_DEV_THRESH: f32 = {{std_dev_thresh}};
const HEIGHTMAP_GRID_WIDTH: NonZeroU32 = {{grid_width}};
const HEIGHTMAP_GRID_HEIGHT: NonZeroU32 = {{grid_height}};
const CELL_COUNT: NonZeroU32 = {{cell_count}};

@compute
@workgroup_size(8, 8, 1)
fn compute_main(@builtin(global_invocation_id) cell: vec3u) {
    let pos = vec2u(cell.x, cell.y);
    
    // Bounds check
    if (pos.x >= HEIGHTMAP_GRID_WIDTH || pos.y >= HEIGHTMAP_GRID_HEIGHT) {
        return;
    }
    
    let idx = xy_to_index(pos);
    
    // Skip if this cell is unknown
    if (atomicLoad(&known_cells[idx]) == 0u) {
        filtered[idx] = height_map_in[idx];
        return;
    }
    
    let current_height = bitcast<f32>(height_map_in[idx]);
    
    // Find K nearest neighbors and compute mean distance
    var distances: array<f32, 100>; // Adjust size based on expected K_NEIGHBORS
    var neighbor_count = 0u;
    
    // Search in expanding radius
    let search_radius = u32(sqrt(f32(K_NEIGHBORS)) * 2.0) + 1u;
    
    for (var dy = 0u; dy < search_radius * 2u + 1u; dy++) {
        for (var dx = 0u; dx < search_radius * 2u + 1u; dx++) {
            let nx = i32(pos.x) + i32(dx) - i32(search_radius);
            let ny = i32(pos.y) + i32(dy) - i32(search_radius);
            
            // Skip self and out of bounds
            if ((nx == i32(pos.x) && ny == i32(pos.y)) || 
                nx < 0 || ny < 0 || 
                nx >= i32(HEIGHTMAP_GRID_WIDTH) || 
                ny >= i32(HEIGHTMAP_GRID_HEIGHT)) {
                continue;
            }
            
            let neighbor_idx = xy_to_index(vec2u(u32(nx), u32(ny)));
            
            // Skip unknown cells
            if (atomicLoad(&known_cells[neighbor_idx]) == 0u) {
                continue;
            }
            
            let neighbor_height = bitcast<f32>(height_map_in[neighbor_idx]);
            let dist = abs(current_height - neighbor_height);
            
            distances[neighbor_count] = dist;
            neighbor_count++;
            
            if (neighbor_count >= K_NEIGHBORS) {
                break;
            }
        }
        if (neighbor_count >= K_NEIGHBORS) {
            break;
        }
    }
    
    // If we don't have enough neighbors, skip processing
    if (neighbor_count < 3u) {
        filtered[idx] = height_map_in[idx];
        return;
    }
    
    // Compute mean height of neighbors
    var neighbor_height_sum = 0.0;
    for (var dy = 0u; dy < search_radius * 2u + 1u; dy++) {
        for (var dx = 0u; dx < search_radius * 2u + 1u; dx++) {
            let nx = i32(pos.x) + i32(dx) - i32(search_radius);
            let ny = i32(pos.y) + i32(dy) - i32(search_radius);
            
            if ((nx == i32(pos.x) && ny == i32(pos.y)) || 
                nx < 0 || ny < 0 || 
                nx >= i32(HEIGHTMAP_GRID_WIDTH) || 
                ny >= i32(HEIGHTMAP_GRID_HEIGHT)) {
                continue;
            }
            
            let neighbor_idx = xy_to_index(vec2u(u32(nx), u32(ny)));
            
            if (atomicLoad(&known_cells[neighbor_idx]) == 0u) {
                continue;
            }
            
            neighbor_height_sum += bitcast<f32>(height_map_in[neighbor_idx]);
            
            if (neighbor_count >= K_NEIGHBORS) {
                break;
            }
        }
        if (neighbor_count >= K_NEIGHBORS) {
            break;
        }
    }
    
    let mean_neighbor_height = neighbor_height_sum / f32(neighbor_count);
    
    // Compute standard deviation of neighbor heights
    var variance_sum = 0.0;
    var checked = 0u;
    for (var dy = 0u; dy < search_radius * 2u + 1u; dy++) {
        for (var dx = 0u; dx < search_radius * 2u + 1u; dx++) {
            let nx = i32(pos.x) + i32(dx) - i32(search_radius);
            let ny = i32(pos.y) + i32(dy) - i32(search_radius);
            
            if ((nx == i32(pos.x) && ny == i32(pos.y)) || 
                nx < 0 || ny < 0 || 
                nx >= i32(HEIGHTMAP_GRID_WIDTH) || 
                ny >= i32(HEIGHTMAP_GRID_HEIGHT)) {
                continue;
            }
            
            let neighbor_idx = xy_to_index(vec2u(u32(nx), u32(ny)));
            
            if (atomicLoad(&known_cells[neighbor_idx]) == 0u) {
                continue;
            }
            
            let neighbor_height = bitcast<f32>(height_map_in[neighbor_idx]);
            let diff = neighbor_height - mean_neighbor_height;
            variance_sum += diff * diff;
            checked++;
            
            if (checked >= K_NEIGHBORS) {
                break;
            }
        }
        if (checked >= K_NEIGHBORS) {
            break;
        }
    }
    
    let std_dev = sqrt(variance_sum / f32(neighbor_count));
    
    // Check if current point deviates from neighbor mean
    let deviation = abs(current_height - mean_neighbor_height);
    let threshold = STD_DEV_THRESH * std_dev;
    
    // If deviation exceeds threshold, mark as outlier
    if (deviation > threshold) {
        // Replace with mean of neighbors
        var neighbor_height_sum = 0.0;
        var valid_neighbors = 0u;
        
        for (var dy = 0u; dy < search_radius * 2u + 1u; dy++) {
            for (var dx = 0u; dx < search_radius * 2u + 1u; dx++) {
                let nx = i32(pos.x) + i32(dx) - i32(search_radius);
                let ny = i32(pos.y) + i32(dy) - i32(search_radius);
                
                if ((nx == i32(pos.x) && ny == i32(pos.y)) || 
                    nx < 0 || ny < 0 || 
                    nx >= i32(HEIGHTMAP_GRID_WIDTH) || 
                    ny >= i32(HEIGHTMAP_GRID_HEIGHT)) {
                    continue;
                }
                
                let neighbor_idx = xy_to_index(vec2u(u32(nx), u32(ny)));
                
                // Skip unknown cells
                if (atomicLoad(&known_cells[neighbor_idx]) == 0u) {
                    continue;
                }
                
                neighbor_height_sum += bitcast<f32>(height_map_in[neighbor_idx]);
                valid_neighbors++;
                
                if (valid_neighbors >= K_NEIGHBORS) {
                    break;
                }
            }
            if (valid_neighbors >= K_NEIGHBORS) {
                break;
            }
        }
        
        let replacement = neighbor_height_sum / f32(valid_neighbors);
        filtered[idx] = bitcast<u32>(replacement);
    } else {
        // Keep original value
        filtered[idx] = height_map_in[idx];
    }
}

fn xy_to_index(pos: vec2u) -> u32 {
    return pos.y * HEIGHTMAP_GRID_WIDTH + pos.x;
}

fn index_to_xy(index: u32) -> vec2u {
    return vec2u(index % HEIGHTMAP_GRID_WIDTH, index / HEIGHTMAP_GRID_WIDTH);
}
"#
);
