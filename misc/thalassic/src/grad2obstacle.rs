use gputter::build_shader;

build_shader!(
    pub(crate) Grad2Obstacle,
    r#"
    const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
    const CELL_COUNT: NonZeroU32 = {{cell_count}};

    // Shader is read_write as it is written to in another shader
    #[buffer] var<storage, read_write> obstacle_map: array<u32, CELL_COUNT>;
    #[buffer] var<storage, read_write> gradient_map: array<f32, CELL_COUNT>;
    #[buffer] var<storage, read_write> height_map: array<f32, CELL_COUNT>;
    #[buffer] var<uniform> max_gradient: f32;

    @compute
    @workgroup_size(8, 8, 1)
    fn grad(
        @builtin(global_invocation_id) global_invocation_id : vec3u,
    ) {
        let index = global_invocation_id.y * HEIGHTMAP_WIDTH + global_invocation_id.x;
        if (global_invocation_id.x >= HEIGHTMAP_WIDTH - 1 || global_invocation_id.x == 0 || global_invocation_id.y == 0 || global_invocation_id.y >= CELL_COUNT / HEIGHTMAP_WIDTH - 1) {
            obstacle_map[index] = 1u;
            return;
        }
        for (var y = -1; y < 2; y++) {
            for (var x = -1; x < 2; x++) {
                let new_index = i32(index) + y * i32(HEIGHTMAP_WIDTH) + x;
                if (height_map[new_index] == 0.0) {
                    obstacle_map[index] = 0u;
                    return;
                }
            }
        }
        if (gradient_map[index] > max_gradient) {
            obstacle_map[index] = 1u;
        } else {
            obstacle_map[index] = 0u;
        }
    }
    "#
);
