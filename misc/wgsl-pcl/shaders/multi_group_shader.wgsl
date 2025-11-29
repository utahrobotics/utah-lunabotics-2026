// Multi-group shader demonstrating complex binding scenarios
// Group 0: Configuration and parameters (uniform buffers)
// Group 1: Data buffers (storage buffers)

// Group 0 - Configuration parameters
struct Config {
    scale_factor: f32,
    offset: f32,
    mode: u32,
    _padding: u32,  // Align to 16 bytes
}

struct Weights {
    weight_a: f32,
    weight_b: f32,
    weight_c: f32,
    _padding: f32,  // Align to 16 bytes
}

@group(0) @binding(0) var<uniform> config: Config;
@group(0) @binding(1) var<uniform> weights: Weights;

// Group 1 - Data buffers
@group(1) @binding(0) var<storage, read> input_a: array<f32>;
@group(1) @binding(1) var<storage, read> input_b: array<f32>;
@group(1) @binding(2) var<storage, read_write> output: array<f32>;

@compute
@workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    let total = arrayLength(&input_a);

    if (index >= total) {
        return;
    }

    // Read from both input buffers
    let a = input_a[index];
    let b = input_b[index];

    // Apply weights from uniform buffer
    var weighted_sum = a * weights.weight_a + b * weights.weight_b;

    // Apply configuration from uniform buffer
    var result: f32;
    
    // Different modes based on config
    if (config.mode == 0u) {
        // Mode 0: Simple weighted sum with scale and offset
        result = (weighted_sum * config.scale_factor) + config.offset;
    } else if (config.mode == 1u) {
        // Mode 1: Product of inputs with scaling
        result = (a * b * config.scale_factor) + config.offset;
    } else {
        // Mode 2: Average with weight_c influence
        result = ((a + b) * 0.5 + weights.weight_c) * config.scale_factor;
    }

    output[index] = result;
}


