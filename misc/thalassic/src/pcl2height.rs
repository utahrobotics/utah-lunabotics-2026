use gputter::build_shader;

build_shader!(
    pub(crate) Pcl2HeightV2,
    r#"
    #[buffer] var<storage, read_write> heightmap: array<f32, CELL_COUNT>;
    #[buffer] var<storage, read_write> points: array<vec3f, MAX_POINT_COUNT>;
    #[buffer] var<storage, read> sorted_triangle_indices: array<vec3u, MAX_TRIANGLE_COUNT>;
    #[buffer] var<uniform> triangle_count: u32;
    
    const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
    const CELL_SIZE: f32 = {{cell_size}};
    const CELL_COUNT: NonZeroU32 = {{cell_count}};
    const MAX_POINT_COUNT: NonZeroU32 = {{max_point_count}}; /!/ sub with 32
    const MAX_TRIANGLE_COUNT: NonZeroU32 = {{max_triangle_count}}; /!/ sub with 32
    
    // Provided by o1-preview
    fn barycentric_coords(p: vec2f, a: vec2f, b: vec2f, c: vec2f) -> vec3f {
        let v0 = b - a;
        let v1 = c - a;
        let v2 = p - a;
        let d00 = dot(v0, v0);
        let d01 = dot(v0, v1);
        let d11 = dot(v1, v1);
        let d20 = dot(v2, v0);
        let d21 = dot(v2, v1);

        let denom = d00 * d11 - d01 * d01;
        let v = (d11 * d20 - d01 * d21) / denom;
        let w = (d00 * d21 - d01 * d20) / denom;
        let u = 1.0 - v - w;
        return vec3f(u, v, w);
    }
    
    @compute
    @workgroup_size(8, 8, 1)
    fn height(
        @builtin(global_invocation_id) global_invocation_id : vec3u,
    ) {
        for (var i = 0u; i < triangle_count; i++) {
            let pp = vec2(- f32(global_invocation_id.x), - f32(global_invocation_id.y)) * CELL_SIZE;
            let tri_indices = sorted_triangle_indices[i];
            let v1 = points[tri_indices.x];
            let v2 = points[tri_indices.y];
            let v3 = points[tri_indices.z];
            let bc = barycentric_coords(pp, vec2(v1.x, v1.y), vec2(v2.x, v2.y), vec2(v3.x, v3.y));
            if (bc.x < 0.0 || bc.y < 0.0 || bc.z < 0.0) {
                continue;
            }
            heightmap[global_invocation_id.y * HEIGHTMAP_WIDTH + global_invocation_id.x] = 
                bc.x * v1.z + 
                bc.y * v2.z + 
                bc.z * v3.z;
            return;
        }
    }
"#
);
