use gputter::build_shader;
build_shader!(
    pub(crate) AdjustOccupancyForCameraPosition,
    r#"
    const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
    const CELL_COUNT: NonZeroU32 = {{cell_count}};
    const CELL_SIZE: f32 = {{cell_size}};
    #[buffer] var<storage, read_write> obstacle_map: array<atomic<u32>, CELL_COUNT>;
    #[buffer] var<storage, read_write> points: array<vec4f>;
    #[buffer] var<uniform> image_dimensions: vec2u;
    #[buffer] var<uniform> camera_transform: mat4x4f;
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
        let camera_inverse = transpose(mat3x3f(
            camera_transform[0].xyz,
            camera_transform[1].xyz,
            camera_transform[2].xyz
        ));
        let camera_translation = camera_transform[3].xyz;
        let camera_space_point = camera_inverse * (point.xyz - camera_translation);
        let distance_from_camera = length(camera_space_point);
        let weight = pow(distance_from_camera/3.2, 2);
        let x_index = u32(point.x / CELL_SIZE);
        let y_index = u32(point.y / CELL_SIZE);
        if (x_index >= HEIGHTMAP_WIDTH || y_index >= (CELL_COUNT / HEIGHTMAP_WIDTH)) {
            return;
        }
        let cell_index = y_index * HEIGHTMAP_WIDTH + x_index;
        let weighted_value = (f32(obstacle_map[cell_index]) * weight) + 10.0;
        atomicStore(&obstacle_map[cell_index], u32(weighted_value));
    }
    "#
);
