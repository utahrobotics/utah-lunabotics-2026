use gputter::build_shader;
build_shader!(
    pub(crate) Pcl2Occupancy,
    r#"
    const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
    const CELL_COUNT: NonZeroU32 = {{cell_count}};
    const CELL_SIZE: f32 = {{cell_size}};

    #[buffer] var<storage, read_write> obstacle_map: array<atomic<u32>, CELL_COUNT>;
    // maps to already processed cells, this flag is set in the final shader
    #[buffer] var<storage, read_write> is_known: array<atomic<u32>, CELL_COUNT>;
    #[buffer] var<storage, read_write> points: array<vec4f>;
    #[buffer] var<uniform> image_dimensions: vec2u;

    @compute
    @workgroup_size(8, 8, 1)
    fn occupancy(
        @builtin(global_invocation_id) global_invocation_id : vec3u,
    ) {
        if (global_invocation_id.x >= image_dimensions.x || global_invocation_id.y >= image_dimensions.y) {
            return;
        }

        let index = global_invocation_id.x + global_invocation_id.y * image_dimensions.x;
        let point = points[index];

        if (point.w == 0.0) {
            return;
        }

        points[index] = vec4f(point.x, point.y, 0.0, point.w);
        let flattened_point = points[index];

        if (flattened_point.x < 0.0 || flattened_point.y < 0.0) {
            return;
        }

        let x_index = u32(flattened_point.x / CELL_SIZE);
        let y_index = u32(flattened_point.y / CELL_SIZE);

        if (x_index >= HEIGHTMAP_WIDTH || y_index >= CELL_COUNT / HEIGHTMAP_WIDTH) {
            return;
        }

        let cell_index = y_index * HEIGHTMAP_WIDTH + x_index;


        atomicAdd(&obstacle_map[cell_index], 1u);
    }
    "#
);
