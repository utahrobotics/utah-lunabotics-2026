use std::vec;

use wgpu::{BindingType, BufferBindingType, ShaderStages};
use wgsl_pcl::mem_layouts::{BindGroupLayoutBuilder, GpuBuffer};
use wgsl_pcl::shader_module_descriptors_from_wgsl;
use wgsl_pcl::shader_pipeline::{
    ComputePipelineBuilder, ComputePipelineChainBuilder, PipelineStage,
};
use wgsl_pcl::wgsl_setup;

#[test]
fn test_shader_module_descriptors_from_wgsl() {
    let descriptors = shader_module_descriptors_from_wgsl!("../shaders/test_shader1.wgsl");

    assert_eq!(descriptors.len(), 1);
}

#[test]
fn test_buffer_read_write() {
    // Initialize GPU
    wgsl_setup::init_gpu_blocking().expect("Failed to initialize GPU");
    let device = wgsl_setup::get_device();

    // Create input data
    let input_data: Vec<u32> = (0..1024).collect();

    // Create input buffer with data
    let input_buffer = GpuBuffer::new_storage_with_data(device, &input_data, Some("input_buffer"));

    // Create output buffer
    let output_buffer = GpuBuffer::new_storage(
        device,
        (input_data.len() * std::mem::size_of::<u32>()) as u64,
        Some("output_buffer"),
    );

    // Create bind group layout
    let (bind_group_layout, bind_group) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            input_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            output_buffer.as_entire_binding(),
        )
        .build(device, "bind_group_layout".to_string());

    // Load shader and create pipeline
    let shader_descriptor = wgpu::include_wgsl!("../shaders/test_shader1.wgsl");
    let pipeline = ComputePipelineBuilder::new(device)
        .with_shader_module(shader_descriptor)
        .with_bind_group_layout(&bind_group_layout)
        .with_entry_point("main")
        .with_label("test_pipeline")
        .build()
        .expect("Failed to create pipeline");

    // Execute the compute shader
    // Workgroup size is 64, so we need (1024 / 64) = 16 workgroups
    pipeline.execute_blocking(device, &[&bind_group], (16, 1, 1));

    // Read back the results
    let output_data: Vec<u32> = output_buffer
        .read_data_blocking(device)
        .expect("Failed to read buffer");

    // Verify the output matches the input
    assert_eq!(output_data.len(), input_data.len());
    for (i, (input, output)) in input_data.iter().zip(output_data.iter()).enumerate() {
        assert_eq!(input, output, "Mismatch at index {}", i);
    }

    println!("✓ Buffer read/write test passed!");
}

#[test]
fn test_buffer_write_and_read() {
    // Initialize GPU
    wgsl_setup::init_gpu_blocking().expect("Failed to initialize GPU");
    let device = wgsl_setup::get_device();

    // Create a buffer
    let buffer = GpuBuffer::new_storage(
        device,
        (100 * std::mem::size_of::<f32>()) as u64,
        Some("test_buffer"),
    );

    // Write data to the buffer
    let write_data: Vec<f32> = (0..100).map(|i| i as f32 * 0.5).collect();
    buffer.write_data(device, &write_data, 0);

    // Read the data back
    let read_data: Vec<f32> = buffer
        .read_data_blocking(device)
        .expect("Failed to read buffer");

    // Verify
    assert_eq!(read_data.len(), write_data.len());
    for (i, (write, read)) in write_data.iter().zip(read_data.iter()).enumerate() {
        assert_eq!(write, read, "Mismatch at index {}", i);
    }

    println!("✓ Buffer write and read test passed!");
}

#[test]
fn test_multi_stage_compute_pipeline() {
    // Initialize GPU
    wgsl_setup::init_gpu_blocking().expect("Failed to initialize GPU");
    let device = wgsl_setup::get_device();

    println!("\n=== Multi-Stage Compute Pipeline Test ===");
    println!("Testing 3-stage pipeline: multiply by 2 -> add 10 -> square");

    // Create input data: [0, 1, 2, 3, ..., 255]
    let input_data: Vec<u32> = (0..256).collect();
    let buffer_size = (input_data.len() * std::mem::size_of::<u32>()) as u64;

    // Create buffers for each stage
    // Input -> Stage 1 -> Intermediate 1 -> Stage 2 -> Intermediate 2 -> Stage 3 -> Output
    let input_buffer = GpuBuffer::new_storage_with_data(device, &input_data, Some("input_buffer"));

    let intermediate_buffer_1 =
        GpuBuffer::new_storage(device, buffer_size, Some("intermediate_buffer_1"));

    let intermediate_buffer_2 =
        GpuBuffer::new_storage(device, buffer_size, Some("intermediate_buffer_2"));

    let output_buffer = GpuBuffer::new_storage(device, buffer_size, Some("output_buffer"));

    // Create shared bind group layout for all stages (all use same layout)
    let (bind_group_layout, bind_group_stage1) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            input_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            intermediate_buffer_1.as_entire_binding(),
        )
        .build(device, "shared_bind_group_layout".to_string());

    // ===== STAGE 1: Multiply by 2 =====
    println!("Creating Stage 1: Multiply by 2");
    let multiply_shader = wgpu::include_wgsl!("../shaders/multiply_shader.wgsl");
    let multiply_pipeline = ComputePipelineBuilder::new(device)
        .with_shader_module(multiply_shader)
        .with_bind_group_layout(&bind_group_layout)
        .with_entry_point("main")
        .with_label("multiply_pipeline")
        .build()
        .expect("Failed to create multiply pipeline");

    // ===== STAGE 2: Add 10 =====
    println!("Creating Stage 2: Add 10");
    let add_shader = wgpu::include_wgsl!("../shaders/add_shader.wgsl");
    let add_pipeline = ComputePipelineBuilder::new(device)
        .with_shader_module(add_shader)
        .with_bind_group_layout(&bind_group_layout)
        .with_entry_point("main")
        .with_label("add_pipeline")
        .build()
        .expect("Failed to create add pipeline");

    let (bind_group_layout_stage2, bind_group_stage2) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            intermediate_buffer_1.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            intermediate_buffer_2.as_entire_binding(),
        )
        .build(device, "bind_group_layout_stage2".to_string());

    // ===== STAGE 3: Square =====
    println!("Creating Stage 3: Square");
    let square_shader = wgpu::include_wgsl!("../shaders/square_shader.wgsl");
    let square_pipeline = ComputePipelineBuilder::new(device)
        .with_shader_module(square_shader)
        .with_bind_group_layout(&bind_group_layout_stage2)
        .with_entry_point("main")
        .with_label("square_pipeline")
        .build()
        .expect("Failed to create square pipeline");

    let (bind_group_layout_stage3, bind_group_stage3) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            intermediate_buffer_2.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            output_buffer.as_entire_binding(),
        )
        .build(device, "bind_group_layout_stage3".to_string());

    // Build the multi-stage pipeline chain
    // Workgroup size is 64, so we need (256 / 64) = 4 workgroups
    let workgroups = (4, 1, 1);

    println!("Building pipeline chain...");
    let pipeline_chain = ComputePipelineChainBuilder::new()
        .add_stage(
            PipelineStage::new(multiply_pipeline, vec![&bind_group_stage1], workgroups)
                .with_label("Stage 1: Multiply".to_string()),
        )
        .add_stage(
            PipelineStage::new(add_pipeline, vec![&bind_group_stage2], workgroups)
                .with_label("Stage 2: Add".to_string()),
        )
        .add_stage(
            PipelineStage::new(square_pipeline, vec![&bind_group_stage3], workgroups)
                .with_label("Stage 3: Square".to_string()),
        )
        .build()
        .expect("Failed to build pipeline chain");

    println!(
        "Executing {}-stage pipeline on GPU (all data stays on GPU)...",
        pipeline_chain.stage_count()
    );
    pipeline_chain.execute_blocking(device);

    // Read back the final results
    println!("Reading results from GPU...");
    let output_data: Vec<u32> = output_buffer
        .read_data_blocking(device)
        .expect("Failed to read output buffer");

    // Verify results: ((input * 2) + 10)^2
    println!("Verifying results...");
    assert_eq!(output_data.len(), input_data.len());

    let mut all_correct = true;
    for (i, (&input, &output)) in input_data.iter().zip(output_data.iter()).enumerate() {
        let expected = {
            let step1 = input * 2;
            let step2 = step1 + 10;
            step2 * step2
        };

        if output != expected {
            println!(
                "  Mismatch at index {}: input={}, expected={}, got={}",
                i, input, expected, output
            );
            all_correct = false;
        }

        // Show first few results for verification
        if i < 5 {
            println!(
                "  [{}] input={} -> *2={} -> +10={} -> ^2={} ✓",
                i,
                input,
                input * 2,
                input * 2 + 10,
                expected
            );
        }
    }

    assert!(all_correct, "Some values didn't match expected results");

    println!("\n✓ Multi-stage compute pipeline test passed!");
    println!("  Successfully executed 3 compute shaders in sequence");
    println!("  All data stayed on GPU between stages (zero CPU-GPU transfers)");
    println!("  Verified {} elements", input_data.len());
}

#[test]
fn test_multiple_bind_groups_and_uniform_buffers() {
    // Initialize GPU
    wgsl_setup::init_gpu_blocking().expect("Failed to initialize GPU");
    let device = wgsl_setup::get_device();

    println!("\n=== Multiple Bind Groups & Uniform Buffers Test ===");
    println!("Testing shader with 2 bind groups and different buffer types");

    // Define uniform buffer structs matching the shader
    #[repr(C)]
    #[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
    struct Config {
        scale_factor: f32,
        offset: f32,
        mode: u32,
        _padding: u32,
    }

    #[repr(C)]
    #[derive(Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
    struct Weights {
        weight_a: f32,
        weight_b: f32,
        weight_c: f32,
        _padding: f32,
    }

    // Test data
    let data_size = 128;
    let input_a: Vec<f32> = (0..data_size).map(|i| i as f32).collect();
    let input_b: Vec<f32> = (0..data_size).map(|i| (i as f32) * 0.5).collect();

    println!("Data size: {} elements", data_size);

    // Create configuration uniform buffer (Group 0, Binding 0)
    let config = Config {
        scale_factor: 2.0,
        offset: 10.0,
        mode: 0,
        _padding: 0,
    };

    let config_buffer = GpuBuffer::new_uniform(
        device,
        std::mem::size_of::<Config>() as u64,
        Some("config_uniform"),
    );
    config_buffer.write_data(device, &[config], 0);
    println!(
        "✓ Created config uniform buffer (scale={}, offset={}, mode={})",
        config.scale_factor, config.offset, config.mode
    );

    // Create weights uniform buffer (Group 0, Binding 1)
    let weights = Weights {
        weight_a: 0.6,
        weight_b: 0.4,
        weight_c: 5.0,
        _padding: 0.0,
    };

    let weights_buffer = GpuBuffer::new_uniform(
        device,
        std::mem::size_of::<Weights>() as u64,
        Some("weights_uniform"),
    );
    weights_buffer.write_data(device, &[weights], 0);
    println!(
        "✓ Created weights uniform buffer (a={}, b={}, c={})",
        weights.weight_a, weights.weight_b, weights.weight_c
    );

    // Create storage buffers (Group 1)
    let input_a_buffer =
        GpuBuffer::new_storage_with_data(device, &input_a, Some("input_a_storage"));
    println!("✓ Created input_a storage buffer");

    let input_b_buffer =
        GpuBuffer::new_storage_with_data(device, &input_b, Some("input_b_storage"));
    println!("✓ Created input_b storage buffer");

    let output_buffer = GpuBuffer::new_storage(
        device,
        (data_size * std::mem::size_of::<f32>()) as u64,
        Some("output_storage"),
    );
    println!("✓ Created output storage buffer");

    // Create bind group layout for Group 0 (uniform buffers)
    println!("\nCreating bind group layouts...");
    let (group0_layout, group0_bind_group) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            config_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            weights_buffer.as_entire_binding(),
        )
        .build(device, "group0_layout".to_string());
    println!("✓ Created Group 0 layout (uniform buffers)");

    // Create bind group layout for Group 1 (storage buffers)
    let (group1_layout, group1_bind_group) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            input_a_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: true },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            input_b_buffer.as_entire_binding(),
        )
        .with_entry(
            2,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            output_buffer.as_entire_binding(),
        )
        .build(device, "group1_layout".to_string());
    println!("✓ Created Group 1 layout (storage buffers)");

    // Create bind groups
    println!("\nCreating bind groups...");
    let (layout_group0, bind_group_0) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            config_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            BindingType::Buffer {
                ty: BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            weights_buffer.as_entire_binding(),
        )
        .build(device, "bind_group_0".to_string());
    println!("✓ Created bind group 0 (2 uniform buffers)");

    println!("✓ Created bind group 1 (3 storage buffers)");

    // Create compute pipeline with both bind group layouts
    println!("\nCreating compute pipeline...");
    let shader = wgpu::include_wgsl!("../shaders/multi_group_shader.wgsl");
    let pipeline = ComputePipelineBuilder::new(device)
        .with_shader_module(shader)
        .with_bind_group_layout(&group0_layout)
        .with_bind_group_layout(&group1_layout)
        .with_entry_point("main")
        .with_label("multi_group_pipeline")
        .build()
        .expect("Failed to create pipeline");
    println!("✓ Created pipeline with 2 bind groups");

    // Execute the shader
    println!("\nExecuting shader on GPU...");
    let workgroups = ((data_size + 63) / 64) as u32; // Round up to nearest 64
    pipeline.execute_blocking(
        device,
        &[&bind_group_0, &group1_bind_group],
        (workgroups, 1, 1),
    );
    println!("✓ Execution complete");

    // Read back results
    println!("\nReading results from GPU...");
    let output: Vec<f32> = output_buffer
        .read_data_blocking(device)
        .expect("Failed to read output buffer");

    // Verify results
    println!("Verifying results...");
    let mut all_correct = true;
    for i in 0..data_size {
        let a = input_a[i];
        let b = input_b[i];

        // Calculate expected result based on shader logic (mode 0)
        let weighted_sum = a * weights.weight_a + b * weights.weight_b;
        let expected = (weighted_sum * config.scale_factor) + config.offset;

        let result = output[i];
        let diff = (result - expected).abs();

        if diff > 0.001 {
            println!(
                "  ✗ Mismatch at index {}: expected {}, got {} (diff: {})",
                i, expected, result, diff
            );
            all_correct = false;
        }

        // Show first few results
        if i < 5 {
            println!(
                "  [{}] a={:.2}, b={:.2} -> weighted_sum={:.2} -> result={:.2} (expected: {:.2}) ✓",
                i, a, b, weighted_sum, result, expected
            );
        }
    }

    assert!(all_correct, "Some values didn't match expected results");

    println!("\n✓ Multiple bind groups test passed!");
    println!("  Group 0: 2 uniform buffers (config + weights)");
    println!("  Group 1: 3 storage buffers (2 inputs + 1 output)");
    println!("  Verified {} elements", data_size);
    println!("  Successfully tested uniform buffer parameters!");
}
