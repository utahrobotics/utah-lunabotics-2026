use std::collections::HashMap;

use wgpu::{ShaderStages, include_wgsl};

use crate::{
    errors::WgslPclError,
    map_layout,
    mem_layouts::{BindGroupLayoutBuilder, GpuBuffer},
    shader_pipeline::ComputePipelineBuilder,
    wgsl_setup::GpuDevice,
};

pub struct ObstacleExpanderOptions {
    pub expansion_radius_meters: f32,
    pub obstacle_gradient_threshold: f32,
}

/// returns the output buffer for the map after obstacle expansion and the compute pipeline, and the compute pipeline
pub fn new_obstacle_expander_pipeline(
    device: &GpuDevice,
    map_layout: map_layout::MapLayout,
    obstacle_expander_options: ObstacleExpanderOptions,
    gradient_map_buffer: &GpuBuffer,
    height_map_width_buffer: &GpuBuffer,
    height_map_height_buffer: &GpuBuffer,
) -> Result<(GpuBuffer, crate::shader_pipeline::ComputePipeline), WgslPclError> {
    let (map_settings_layout, map_settings_bind_group) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            height_map_width_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            height_map_height_buffer.as_entire_binding(),
        )
        .build(&device, "map_settings_bind_group".to_string());
    let obstacle_expander_constants: HashMap<&str, f64> = HashMap::from([
        (
            "ROBOT_RADIUS_METERS",
            obstacle_expander_options.expansion_radius_meters as f64,
        ),
        ("CELL_SIZE_METERS", map_layout.cell_size as f64),
    ]);
    let obstacle_expander_output_buffer = GpuBuffer::new_storage_with_data(
        &device,
        &vec![
            f32::MIN;
            ((map_layout.width_meters() / map_layout.cell_size).ceil() as usize)
                * ((map_layout.height_meters() / map_layout.cell_size).ceil() as usize)
        ],
        Some("obstacle_expanded_height_map"),
    );
    let max_gradient_buffer = GpuBuffer::new_uniform_with_data(
        &device,
        &obstacle_expander_options.obstacle_gradient_threshold,
        Some("max_gradient_buffer"),
    );
    let (obstacle_expander_layout, obstacle_expander_bind_group) =
        BindGroupLayoutBuilder::new(None)
            .with_entry(
                0,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                gradient_map_buffer.as_entire_binding(),
            )
            .with_entry(
                1,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                obstacle_expander_output_buffer.as_entire_binding(),
            )
            .with_entry(
                2,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                max_gradient_buffer.as_entire_binding(),
            )
            .build(&device, "obstacle_expander_bind_group".to_string());
    let obstacle_expander_pipeline = ComputePipelineBuilder::new(device)
        .with_label("obstacle_expander")
        .with_shader_module(include_wgsl!("../../shaders/expand_obstacles.wgsl"))
        .with_bind_group(0, map_settings_layout, map_settings_bind_group)
        .with_bind_group(1, obstacle_expander_layout, obstacle_expander_bind_group)
        .with_constants(obstacle_expander_constants)
        .build()?;
    Ok((obstacle_expander_output_buffer, obstacle_expander_pipeline))
}
