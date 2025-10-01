use gputter::build_shader;

build_shader!(
    pub(crate) Pcl2HeightV3,
    r#"
#[buffer] var<storage, read_write> heightmap: array<atomic<u32>, CELL_COUNT>;
#[buffer] var<storage, read_write> points: array<vec4f>;
#[buffer] var<uniform> image_dimensions: vec2u;

const HEIGHTMAP_WIDTH: NonZeroU32 = {{heightmap_width}};
const HEIGHTMAP_HEIGHT: NonZeroU32 = {{heightmap_height}};
const CELL_SIZE: f32 = {{cell_size}};
const CELL_COUNT: NonZeroU32 = {{cell_count}};

@compute
@workgroup_size(8, 8, 1)
fn height(
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
    // calculates which cell this point belongs to
    let cell_x = u32(point.x / CELL_SIZE);
    let cell_y = u32(point.y / CELL_SIZE);
    
    // chekcs bounds
    if (cell_x >= u32(HEIGHTMAP_WIDTH) || cell_y >= u32(HEIGHTMAP_HEIGHT)) {
        return;
    }
    
    // calculates cell index using width for row stride
    let cell_index = cell_y * u32(HEIGHTMAP_WIDTH) + cell_x;
    
    // some GPUs dont support atomic floats so we convert it to a u32
    let new_height_bits = bitcast<u32>(point.z);
    
    // hacky loop to ensure thread safety because we can't compare the 2s compliment version of 
    // f32s in a compare and swap or atomicMax operation
    loop {
        let current_height_bits = atomicLoad(&heightmap[cell_index]);
        let current_height = bitcast<f32>(current_height_bits);
        
        if (point.z > current_height) {
            let exchanged = atomicCompareExchangeWeak(&heightmap[cell_index], current_height_bits, new_height_bits);
            if (exchanged.exchanged) {
                break;
            }
        } else {
            break;
        }
    }
}
"#
);
