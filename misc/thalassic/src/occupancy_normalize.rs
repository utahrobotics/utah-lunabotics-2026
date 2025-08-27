use gputter::build_shader;
build_shader!(
    pub(crate) OccupancyNormalize,
    r#"
        const CELL_COUNT: NonZeroU32 = {{cell_count}};
        const GRID_WIDTH: NonZeroU32 = {{grid_width}};
        const GRID_HEIGHT: NonZeroU32 = {{grid_height}};
        
        // Input: raw occupancy counts from previous shader
        #[buffer] var<storage, read_write> raw_occupancy: array<u32, CELL_COUNT>;
        // Output: normalized occupancy scores (0-10)
        #[buffer] var<storage, read_write> normalized_occupancy: array<u32, CELL_COUNT>;
        #[buffer] var<storage, read_write> is_known: array<atomic<u32>, CELL_COUNT>;
        #[buffer] var<uniform> min_points_for_occupied: u32;
        #[buffer] var<uniform> max_points_threshold: u32;
        #[buffer] var<uniform> neighborhood_radius: u32;
        #[buffer] var<uniform> min_known_neighbors_ratio: u32;
        
        @compute
        @workgroup_size(8, 8, 1)
        fn normalize(
            @builtin(global_invocation_id) global_invocation_id : vec3u,
        ) {
            if (global_invocation_id.x >= GRID_WIDTH || global_invocation_id.y >= GRID_HEIGHT) {
                return;
            }
            
            let x = global_invocation_id.x;
            let y = global_invocation_id.y;
            let cell_index = y * GRID_WIDTH + x;
            let raw_count = raw_occupancy[cell_index];
            let is_cell_known = atomicLoad(&is_known[cell_index]) > 0u;
            
            // mark as unknown (0)
            if (raw_count == 0u) {
                // known cell with a positive score, keep the existing score
                if (is_cell_known && normalized_occupancy[cell_index] > 0u) {
                    return;
                }
                normalized_occupancy[cell_index] = 0u;
                return;
            }
            
            if (is_cell_known && normalized_occupancy[cell_index] > 0u) {
                return;
            }
            
            // Check neighborhood for unknown neighbors
            var total_neighbors = 0u;
            var known_neighbors = 0u;
            var local_max = raw_count;
            var local_sum = raw_count;
            var local_count = 1u;
            
            for (var dy = -i32(neighborhood_radius); dy <= i32(neighborhood_radius); dy++) {
                for (var dx = -i32(neighborhood_radius); dx <= i32(neighborhood_radius); dx++) {
                    if (dx == 0 && dy == 0) {
                        continue; // Skip the center cell
                    }
                    
                    let nx = i32(x) + dx;
                    let ny = i32(y) + dy;
                    if (nx < 0 || ny < 0 || nx >= i32(GRID_WIDTH) || ny >= i32(GRID_HEIGHT)) {
                        continue;
                    }
                    
                    let neighbor_index = u32(ny) * GRID_WIDTH + u32(nx);
                    let neighbor_count = raw_occupancy[neighbor_index];
                    let neighbor_is_known = atomicLoad(&is_known[neighbor_index]) > 0u;
                    
                    total_neighbors += 1u;
                    
                    if (neighbor_is_known || neighbor_count > 0u) {
                        known_neighbors += 1u;
                        
                        if (neighbor_count > 0u) {
                            local_max = max(local_max, neighbor_count);
                            local_sum += neighbor_count;
                            local_count += 1u;
                        }
                    }
                }
            }
            
            // Check if we have sufficient known neighbors
            if (total_neighbors > 0u) {
                let known_ratio = (known_neighbors * 100u) / total_neighbors;
                if (known_ratio < min_known_neighbors_ratio) {
                    // Too many unknown neighbors
                    normalized_occupancy[cell_index] = 0u;
                    return;
                }
            }
            
            let local_avg = local_sum / local_count;
            let effective_max = max(max_points_threshold, local_max);
            
            var count_score = 0u;
            if (raw_count >= min_points_for_occupied) {
                count_score = min(6u, 1u + (raw_count * 5u) / effective_max);
            } else {
                count_score = 1u;
            }
            
            var density_bonus = 0u;
            if (raw_count > local_avg) {
                density_bonus = min(2u, (raw_count - local_avg) / max(1u, local_avg / 2u));
            }
            
            var relative_bonus = 0u;
            if (raw_count >= effective_max / 2u) {
                relative_bonus = 1u;
                if (raw_count >= (effective_max * 3u) / 4u) {
                    relative_bonus = 2u;
                }
            }
            
            let final_score = min(10u, count_score + density_bonus + relative_bonus);
            normalized_occupancy[cell_index] = final_score;
            
            if final_score > 0 {
                atomicAdd(&is_known[cell_index], 1u);
            }
        }
    "#
);
