use gputter::build_shader;

build_shader!(
    pub(crate) Pcl2Obstacle,
    r#"
    const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
    const CELL_COUNT: NonZeroU32 = {{cell_count}};
    const CELL_SIZE: f32 = {{cell_size}};

    // Shader is read_write as it is written to in another shader
    #[buffer] var<storage, read_write> obstacle_map: array<atomic<u32>, CELL_COUNT>;
    #[buffer] var<storage, read_write> points: array<vec4f>;
    #[buffer] var<uniform> max_safe_gradient: f32;
    #[buffer] var<uniform> image_dimensions: vec2u;

    @compute
    @workgroup_size(8, 8, 1)
    fn grad(
        @builtin(global_invocation_id) global_invocation_id : vec3u,
    ) {
        if (global_invocation_id.x >= image_dimensions.x - 1 || global_invocation_id.x == 0 || global_invocation_id.y == 0 || global_invocation_id.y >= image_dimensions.y - 1) {
            return;
        }
        let index = global_invocation_id.x + global_invocation_id.y * image_dimensions.x;
        let origin = points[index];

        if (origin.x < 0.0 || origin.y < 0.0 || origin.w == 0.0) {
            return;
        }

        let x_index = u32(origin.x / CELL_SIZE);
        let y_index = u32(origin.y / CELL_SIZE);

        if (x_index >= HEIGHTMAP_WIDTH || y_index >= CELL_COUNT / HEIGHTMAP_WIDTH) {
            return;
        }

        let points = array<vec4f, 8>(
            points[index + 1],
            points[index + 1 - image_dimensions.x],
            points[index - image_dimensions.x],
            points[index - 1 - image_dimensions.x],
            points[index - 1],
            points[index - 1 + image_dimensions.x],
            points[index + image_dimensions.x],
            points[index + 1 + image_dimensions.x],
        );
        var sum = vec3f(0.0, 0.0, 0.0);
        var count = 0;
        var crosses = array<vec4f, 8>();
        for (var i = 0; i < 8; i++) {
            let next_i = (i + 1) % 8;
            if (points[i].w == 0.0 || points[next_i].w == 0.0) {
                continue;
            }
            let v1 = points[i].xyz - origin.xyz;
            let v2 = points[next_i].xyz - origin.xyz;
            let cross = normalize(cross(v1, v2));
            crosses[i] = vec4f(cross, 1.0);
            sum += cross;
            count += 1;
        }
        if (count < 3) {
            return;
        }
        let normal = normalize(sum);
        var max_gradient = -1.0;
        for (var i = 0; i < 8; i++) {
            if (crosses[i].w == 0.0) {
                continue;
            }
            let gradient = acos(dot(crosses[i].xyz, normal));
            if (gradient > max_gradient) {
                max_gradient = gradient;
            }
        }
        let obstacle_index = y_index * HEIGHTMAP_WIDTH + x_index;
        if (max_gradient > max_safe_gradient) {
            atomicStore(&obstacle_map[obstacle_index], 2u);
        } else {
            atomicStore(&obstacle_map[obstacle_index], 1u);
        }
    }
    "#
);
