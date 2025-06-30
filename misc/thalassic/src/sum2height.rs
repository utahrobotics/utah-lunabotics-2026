use gputter::build_shader;

build_shader!(
    pub(crate) Sum2Height,
    r#"
    #[buffer] var<storage, read_write> sum: array<vec2f, CELL_COUNT>;
    #[buffer] var<storage, read_write> heightmap: array<f32, CELL_COUNT>;
    
    const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
    const CELL_COUNT: NonZeroU32 = {{cell_count}};
    const MIN_COUNT: f32 = {{min_count}};
    
    @compute
    @workgroup_size(8, 8, 1)
    fn height(
        @builtin(global_invocation_id) global_invocation_id : vec3u,
    ) {
        let heightmap_index = global_invocation_id.x + global_invocation_id.y * HEIGHTMAP_WIDTH;

        let sum_vec = sum[heightmap_index];
        if (sum_vec.x <= MIN_COUNT) {
            return;
        }
        heightmap[heightmap_index] = sum_vec.y / sum_vec.x;
        sum[heightmap_index] = vec2(0.0, 0.0);
    }
"#
);
