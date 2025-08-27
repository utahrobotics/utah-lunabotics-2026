use gputter::build_shader;

build_shader!(
    pub(crate) ExpandFreeOccupancy,
r#"
#[buffer] var<storage, read_write> expanded_obstacle_map: array<u32>;
#[buffer] var<storage, read_write> final_map: array<u32>;
#[buffer] var<storage, read_write> is_known: array<atomic<u32>>;


const GRID_WIDTH: NonZeroU32 = {{grid_width}};
const GRID_HEIGHT: NonZeroU32 = {{grid_height}};

@compute
@workgroup_size(8, 8, 1)
fn compute_main(@builtin(global_invocation_id) cell: vec3u) {
    let pos = cell.xy;
    let center_i = xy_to_index(pos);
    let radius_in_cells = u32(5);
    if (pos.x >= GRID_WIDTH - radius_in_cells || pos.x <= radius_in_cells || pos.y <= radius_in_cells || pos.y >= GRID_HEIGHT - radius_in_cells) {
        return; // already an obstacle
    }

    if (expanded_obstacle_map[center_i] > 0) { // not unknown
        final_map[center_i] = expanded_obstacle_map[center_i];
        return;
    }

    // convert to i32 temporarily to avoid underflowing to maximum u32 values
    let start_x = u32( max( i32(0), i32(pos.x - radius_in_cells ) ) );
    let end_x = min(GRID_WIDTH-1, pos.x + radius_in_cells);

    let start_y = u32( max( i32(0), i32(pos.y - radius_in_cells ) ) );
    let end_y = min( GRID_HEIGHT-1 , pos.y + radius_in_cells);

    for (var x = start_x; x <= end_x; x++) {
        for (var y = start_y; y <= end_y; y++) {
            if (u32(abs(i32(x) - i32(pos.x))) + u32(abs(i32(y) - i32(pos.y))) > radius_in_cells) {
                continue;
            }
            let i = xy_to_index(vec2u(x, y));
            if (expanded_obstacle_map[i] == 0) {
                final_map[center_i] = 1;
                // atomicAdd(&is_known[center_i], 1u);
                return;
            }
        }
    }

}


fn xy_to_index(pos: vec2u) -> u32 {
    return pos.y * GRID_WIDTH + pos.x;
}
fn index_to_xy(index: u32) -> vec2u {
    return vec2u(index % GRID_WIDTH, index / GRID_WIDTH);
}

"#
);
