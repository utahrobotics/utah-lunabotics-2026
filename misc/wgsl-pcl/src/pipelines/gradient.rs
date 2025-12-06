use std::collections::HashMap;

use wgpu::{ShaderStages, include_wgsl};

use crate::{
    errors::WgslPclError,
    map_layout,
    mem_layouts::{BindGroupLayoutBuilder, GpuBuffer},
    shader_pipeline::ComputePipelineBuilder,
    wgsl_setup::GpuDevice,
};
pub fn new_gradient_pipeline(
    device: &GpuDevice,
    workgroup_size_stage2: (u32, u32),
    map_layout: map_layout::MapLayout,
    gradient_kernel_radius: u32,
    blur_filtered_height_map_buffer: &GpuBuffer,
) -> Result<(GpuBuffer, crate::shader_pipeline::ComputePipeline), WgslPclError> {
    let gradient_map_buffer = GpuBuffer::new_storage_with_data(
        &device,
        &vec![
            0.0f32;
            ((map_layout.width_meters() / map_layout.cell_size).ceil() as usize)
                * ((map_layout.height_meters() / map_layout.cell_size).ceil() as usize)
        ],
        Some("gradient_map"),
    );
    let gradient_constants: HashMap<&str, f64> = HashMap::from([
        (
            "MAP_WIDTH",
            (map_layout.width_meters() / map_layout.cell_size).ceil() as f64,
        ),
        (
            "MAP_HEIGHT",
            (map_layout.height_meters() / map_layout.cell_size).ceil() as f64,
        ),
        ("KERNEL_RADIUS", gradient_kernel_radius as f64),
        ("WORKGROUP_X", workgroup_size_stage2.0 as f64),
        ("WORKGROUP_Y", workgroup_size_stage2.1 as f64),
        ("CELL_SIZE", map_layout.cell_size as f64),
    ]);
    let (gradient_layout, gradient_bind_group) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            blur_filtered_height_map_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            gradient_map_buffer.as_entire_binding(),
        )
        .build(&device, "gradient_bind_group".to_string());
    let gradient_pipeline = ComputePipelineBuilder::new(device)
        .with_label("gradient_mapper")
        .with_shader_module(include_wgsl!("../../shaders/height_map_to_gradients.wgsl"))
        .with_bind_group(0, gradient_layout, gradient_bind_group)
        .with_constants(gradient_constants)
        .build()?;
    Ok((gradient_map_buffer, gradient_pipeline))
}
