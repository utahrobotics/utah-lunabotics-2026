use gputter::build_shader;
// only applies blur to known cells
build_shader!(
    pub(crate) GaussianBlur,
    r#"
#[buffer] var<storage, read_write> heightmap: array<atomic<u32>, CELL_COUNT>;
#[buffer] var<storage, read_write> blurred_heightmap: array<atomic<u32>, CELL_COUNT>;
#[buffer] var<storage, read_write> known_cells: array<atomic<u32>, CELL_COUNT>;


const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
const HEIGHTMAP_HEIGHT: NonZeroU32 = {{heightmap_height}};
const CELL_SIZE: f32 = {{cell_size}};
const CELL_COUNT: NonZeroU32 = {{cell_count}};
const KERNEL_SIZE: u32 = {{kernel_size}};

@compute
@workgroup_size(8, 8, 1)
fn gaussian_blur(
    @builtin(global_invocation_id) global_invocation_id : vec3u,
) {
    if (global_invocation_id.x >= HEIGHTMAP_WIDTH || global_invocation_id.y >= HEIGHTMAP_HEIGHT) {
        return;
    }
    let x = i32(global_invocation_id.x);
    let y = i32(global_invocation_id.y);
    let cell_index = u32(y) * u32(HEIGHTMAP_WIDTH) + u32(x);
    
    // Get current height value (convert from atomic u32 to float)
    let current_height = bitcast<f32>(atomicLoad(&heightmap[cell_index]));
    
    var weighted_sum = 0.0f;
    var weight_sum = 0.0f;
    let half_kernel = i32(KERNEL_SIZE / 2u);
    
    // Apply Gaussian blur kernel
    for (var ky: i32 = -half_kernel; ky <= half_kernel; ky++) {
        for (var kx: i32 = -half_kernel; kx <= half_kernel; kx++) {
            let sample_x = x + kx;
            let sample_y = y + ky;
            let sample_index = u32(sample_y) * u32(HEIGHTMAP_WIDTH) + u32(sample_x);
            // Check bounds, and also check if cell is known.
            if (sample_x >= 0i && sample_x < i32(HEIGHTMAP_WIDTH) && 
                sample_y >= 0i && sample_y < i32(HEIGHTMAP_HEIGHT)) && known_cells[sample_index] == 1 && known_cells[cell_index] == 1 {
                
                
                let sample_height = bitcast<f32>(atomicLoad(&heightmap[sample_index]));
                
                // Gaussian weight calculation
                let distance_sq = f32(kx * kx + ky * ky);
                let sigma = f32(KERNEL_SIZE) / 3.0f; // Standard deviation
                let weight = exp(-distance_sq / (2.0f * sigma * sigma));
                
                weighted_sum += sample_height * weight;
                weight_sum += weight;
            }
        }
    }
    
    // Apply the blur result
    if (weight_sum > 0.0f) {
        let blurred_height = weighted_sum / weight_sum;
        let blurred_height_u32 = bitcast<u32>(blurred_height);
        atomicStore(&blurred_heightmap[cell_index], blurred_height_u32);
    }
}

    "#
);
