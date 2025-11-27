// Stage 1: Multiply each element by 2
@group(0) @binding(0) var<storage, read> input: array<u32>;
@group(0) @binding(1) var<storage, read_write> output: array<u32>;

@compute
@workgroup_size(64)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let index = global_id.x;
    let total = arrayLength(&input);

    if (index >= total) {
        return;
    }

    output[index] = input[index] * 2u;
}

