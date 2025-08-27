use gputter::build_shader;
build_shader!(
    pub(crate) OccupancyNormalize,
    r#"
        const CELL_COUNT: NonZeroU32 = {{cell_count}};
        const GRID_WIDTH: NonZeroU32 = {{grid_width}};
        const GRID_HEIGHT: NonZeroU32 = {{grid_height}};

        // Input: raw occupancy counts from previous shader
        #[buffer] var<storage, read_write> raw_occupancy: array<u32, CELL_COUNT>;
        // Output: normalized occupancy scores (0-100, 0 = unknown)
        #[buffer] var<storage, read_write> normalized_occupancy: array<u32, CELL_COUNT>;
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

            // mark as unknown (0)
            if (raw_count == 0u) {
                // known cell with a positive score, keep the existing score
                if (normalized_occupancy[cell_index] > 0u) {
                    return;
                }
                normalized_occupancy[cell_index] = 0u;
                return;
            }

            // Check neighborhood for unknown neighbors
            var total_neighbors = 0u;
            var known_neighbors = 0u;
            var local_max = raw_count;
            var local_sum = raw_count;
            var local_count = 1u;
            var occupied_neighbors = 0u;

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

                    total_neighbors += 1u;

                    if (neighbor_count > 0u) {
                        known_neighbors += 1u;

                        if (neighbor_count > 0u) {
                            local_max = max(local_max, neighbor_count);
                            local_sum += neighbor_count;
                            local_count += 1u;

                            if (neighbor_count >= min_points_for_occupied) {
                                occupied_neighbors += 1u;
                            }
                        }
                    }
                }
            }

            if (total_neighbors > 0u) {
                let known_ratio = (known_neighbors * 100u) / total_neighbors;
                if (known_ratio < min_known_neighbors_ratio) {
                    // Too many unknown neighbors
                    normalized_occupancy[cell_index] = 0u;
                    return;
                }
            }

            let local_avg = local_sum / local_count;

            // density score (0-40 points)
            // compares to its local neighborhood average
            var relative_density_score = 0u;
            if (local_avg > 0u) {
                let ratio = (raw_count * 100u) / local_avg;
                if (ratio >= 100u) {
                    // Above average
                    relative_density_score = 20u + min(20u, (ratio - 100u) / 10u);
                } else {
                    // Below average but still has points
                    relative_density_score = (ratio * 25u) / 100u;
                }
            }

            // local maximum score
            // how close this cell is to being a local maximum
            var local_max_score = 0u;
            if (local_max > 0u) {
                let max_ratio = (raw_count * 100u) / local_max;
                local_max_score = (max_ratio * 30u) / 100u;
            }

            // spatial consistency score (0-30 points)
            // how many neighbors are also occupied
            var consistency_score = 0u;
            if (total_neighbors > 0u) {
                let occupied_ratio = (occupied_neighbors * 100u) / total_neighbors;
                consistency_score = (occupied_ratio * 30u) / 100u;
            }

            // if we meet the minimum threshold, ensure at least a score of 10
            var threshold_bonus = 0u;
            if (raw_count >= min_points_for_occupied) {
                // this didn't turn out to be helpful
                // threshold_bonus = 10u;
            }

            // Calculate final score (1-100)
            let raw_score = relative_density_score + local_max_score + consistency_score;
            let total_score = max(raw_score, threshold_bonus);
            let final_score = min(100u, max(1u, total_score));

            normalized_occupancy[cell_index] = final_score;
        }
    "#
);
