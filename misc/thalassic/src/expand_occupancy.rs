use gputter::build_shader;

build_shader!(
    pub(crate) ExpandOccupancy,
r#"
#[buffer] var<storage, read_write> normalized_occupancy_grid: array<u32>;
#[buffer] var<storage, read_write> expanded_obstacles: array<u32>;
#[buffer] var<storage, read_write> is_known: array<atomic<u32>>;


// radius of the robot in cells
#[buffer] var<uniform> radius_in_cells: u32;

// a number (1 - 100) n such that any cell with score > n is considered an obstacle
const OBSTACLE_THRESHOLD: NonZeroU32 = {{obstacle_threshold}};

const GRID_WIDTH: NonZeroU32 = {{grid_width}};
const GRID_HEIGHT: NonZeroU32 = {{grid_height}};

@compute
@workgroup_size(8, 8, 1)
fn compute_main(@builtin(global_invocation_id) cell: vec3u) {
    let pos = cell.xy;

    if (pos.x >= GRID_WIDTH || pos.y >= GRID_HEIGHT) {
        return;
    }

    let center_i = xy_to_index(pos);

    if (pos.x < radius_in_cells || pos.x >= GRID_WIDTH - radius_in_cells ||
        pos.y < radius_in_cells || pos.y >= GRID_HEIGHT - radius_in_cells) {
        expanded_obstacles[center_i] = 100u;
        return;
    }

    let original_score = normalized_occupancy_grid[center_i];

    if (original_score == 0u) {
        expanded_obstacles[center_i] = 0u;
        return;
    }

    if (original_score > OBSTACLE_THRESHOLD) {
        expanded_obstacles[center_i] = original_score;
        return;
    }

    var max_influence_score = original_score;

    let start_x = u32(max(i32(0), i32(pos.x) - i32(radius_in_cells)));
    let end_x = min(GRID_WIDTH - 1u, pos.x + radius_in_cells);
    let start_y = u32(max(i32(0), i32(pos.y) - i32(radius_in_cells)));
    let end_y = min(GRID_HEIGHT - 1u, pos.y + radius_in_cells);

    for (var x = start_x; x <= end_x; x++) {
        for (var y = start_y; y <= end_y; y++) {
            let dx = i32(x) - i32(pos.x);
            let dy = i32(y) - i32(pos.y);
            let dist_squared = u32(dx * dx + dy * dy);

            if (dist_squared > radius_in_cells * radius_in_cells) {
                continue;
            }

            let neighbor_i = xy_to_index(vec2u(x, y));
            let neighbor_score = normalized_occupancy_grid[neighbor_i];

            if (neighbor_score > OBSTACLE_THRESHOLD) {
                let distance = sqrt(f32(dist_squared));
                let radius_f = f32(radius_in_cells);

                let decay_factor = pow(1.0 - (distance / radius_f), 2.0);

                let decayed_score = u32(max(50.0, f32(neighbor_score) * decay_factor));

                max_influence_score = max(max_influence_score, decayed_score);
            } else {
                let distance = sqrt(f32(dist_squared));
                let radius_f = f32(radius_in_cells);
                let decay_factor = 1.0 - (distance / radius_f);

                let influence = u32(f32(neighbor_score) * decay_factor * 0.3);
                max_influence_score = max(max_influence_score, original_score + influence);
            }
        }
    }

    expanded_obstacles[center_i] = min(100u, max_influence_score);
}

fn xy_to_index(pos: vec2u) -> u32 {
    return pos.y * GRID_WIDTH + pos.x;
}

fn index_to_xy(index: u32) -> vec2u {
    return vec2u(index % GRID_WIDTH, index / GRID_WIDTH);
}
"#
);
