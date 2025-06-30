use gputter::build_shader;

build_shader!(
    pub(crate) ObstacleFilter,
r#"

#[buffer] var<storage, read_write> in_obstacles: array<u32>;
#[buffer] var<storage, read_write> filtered_obstacles: array<u32>;

const FEATURE_RADIUS_CELLS: u32 = {{feature_size_cells}};
const MIN_COUNT: u32 = {{min_count}};

const CELL_SIZE: f32 = {{cell_size}};

const GRID_WIDTH: NonZeroU32 = {{grid_width}};
const GRID_HEIGHT: NonZeroU32 = {{grid_height}};

@compute
@workgroup_size(8, 8, 1)
fn compute_main(@builtin(global_invocation_id) cell: vec3u) {
    let pos = cell.xy;
    let this_index = xy_to_index(pos);

    if (in_obstacles[this_index] == 0) {
        filtered_obstacles[this_index] = 0;
        return;
    }

    if (cell.x >= GRID_WIDTH - 1 || cell.x == 0 || cell.y == 0 || cell.y >= GRID_HEIGHT - 1) {
        filtered_obstacles[this_index] = 2u;
        return;
    }

    // convert to i32 temporarily to avoid underflowing to maximum u32 values
    let start_x = u32( max( i32(0), i32(pos.x - FEATURE_RADIUS_CELLS ) ) );
    let end_x = min(GRID_WIDTH-1, pos.x + FEATURE_RADIUS_CELLS);
    
    let start_y = u32( max( i32(0), i32(pos.y - FEATURE_RADIUS_CELLS ) ) );
    let end_y = min( GRID_HEIGHT-1 , pos.y + FEATURE_RADIUS_CELLS);
    var count = 0u;

    for (var x = start_x; x <= end_x; x++) {
        for (var y = start_y; y <= end_y; y++) {

            let nearby_pos = vec2(x, y);
            let nearby_i = xy_to_index(nearby_pos);

            if (in_obstacles[nearby_i] == 2) {
                count += 1;

                if (count >= MIN_COUNT) {
                    filtered_obstacles[this_index] = 2;
                    return;
                }
            }
        }
    }

    filtered_obstacles[this_index] = 1;
}


fn xy_to_index(pos: vec2u) -> u32 {
    return pos.y * GRID_WIDTH + pos.x;
}
fn index_to_xy(index: u32) -> vec2u {
    return vec2u(index % GRID_WIDTH, index / GRID_WIDTH);
} 

"#
);
