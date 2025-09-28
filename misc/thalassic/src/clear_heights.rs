use gputter::build_shader;
// Clear Heights sets cells in the heightmap that will be recalculated with the new point cloud
// to a very small number. This prevents the height map from just forever increasing as more point clouds come in
// it might be more efficient to just do this step on the CPU, I'm not sure.
build_shader!(
    pub(crate) ClearHeights,
    r#"
    const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
    const CELL_COUNT: NonZeroU32 = {{cell_count}};
    const CELL_SIZE: f32 = {{cell_size}};

    #[buffer] var<storage, read_write> height_map: array<atomic<u32>, CELL_COUNT>;
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

        // Check for negative coordinates that would cause issues
        if (point.x < 0.0 || point.y < 0.0) {
            return;
        }

        let x_index = u32(point.x / CELL_SIZE);
        let y_index = u32(point.y / CELL_SIZE);
        if (x_index >= HEIGHTMAP_WIDTH || y_index >= CELL_COUNT / HEIGHTMAP_WIDTH) {
            return;
        }
        let cell_index = y_index * HEIGHTMAP_WIDTH + x_index;

        atomicStore(&height_map[cell_index], bitcast<u32>(-9999.0));
    }
    "#
);
