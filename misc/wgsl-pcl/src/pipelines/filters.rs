use std::collections::HashMap;

use wgpu::{ShaderStages, include_wgsl};

use crate::{
    errors::WgslPclError,
    map_layout,
    mem_layouts::{BindGroupLayoutBuilder, GpuBuffer},
    shader_pipeline::ComputePipelineBuilder,
    wgsl_setup::GpuDevice,
};

pub struct BilateralOptions {
    pub sigma_spatial: f32,
    pub sigma_range: f32,
    pub kernel_radius: u32,
}

pub struct GaussianOptions {
    pub kernel_radius: u32,
    pub sigma: f32,
}

pub enum BlurFilterOptions {
    Bilateral(BilateralOptions),
    Gaussian(GaussianOptions),
}

pub struct OutlierFilterOptions {
    pub kernel_radius: u32,
    pub std_dev_threshold: f32,
}

/// returns the output buffer for the map after outlier removal and the compute pipeline
pub fn new_outlier_removal_pipeline(
    device: &GpuDevice,
    map_layout: map_layout::MapLayout,
    outlier_filter_options: OutlierFilterOptions,
    input_height_map_buffer: &GpuBuffer,
) -> Result<(GpuBuffer, crate::shader_pipeline::ComputePipeline), WgslPclError> {
    let outlier_filtered_height_map_buffer = GpuBuffer::new_storage_with_data(
        &device,
        &vec![
            f32::MIN;
            ((map_layout.width_meters() / map_layout.cell_size).ceil() as usize)
                * ((map_layout.height_meters() / map_layout.cell_size).ceil() as usize)
        ],
        Some("outlier_filtered_height_map"),
    );
    let outlier_filter_constants: HashMap<&str, f64> = HashMap::from([
        (
            "MAP_WIDTH",
            (map_layout.width_meters() / map_layout.cell_size).ceil() as f64,
        ),
        (
            "MAP_HEIGHT",
            (map_layout.height_meters() / map_layout.cell_size).ceil() as f64,
        ),
        ("KERNEL_RADIUS", outlier_filter_options.kernel_radius as f64),
        (
            "OUTLIER_THRESHOLD",
            outlier_filter_options.std_dev_threshold as f64,
        ),
    ]);
    let (outlier_filter_layout, outlier_filter_bind_group) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            input_height_map_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            outlier_filtered_height_map_buffer.as_entire_binding(),
        )
        .build(&device, "outlier_filter_bind_group".to_string());
    let outlier_filter_pipeline = ComputePipelineBuilder::new(device)
        .with_label("outlier_filter")
        .with_shader_module(include_wgsl!("../../shaders/outlier_removal.wgsl"))
        .with_bind_group(0, outlier_filter_layout, outlier_filter_bind_group)
        .with_constants(outlier_filter_constants)
        .build()?;
    Ok((outlier_filtered_height_map_buffer, outlier_filter_pipeline))
}

/// returns the output buffer for the blurred map and the compute pipeline
pub fn new_blur_pipeline(
    device: &GpuDevice,
    workgroup_size_stage2: (u32, u32),
    map_layout: map_layout::MapLayout,
    blur_filter_options: BlurFilterOptions,
    input_height_map_buffer: &GpuBuffer,
) -> Result<(GpuBuffer, crate::shader_pipeline::ComputePipeline), WgslPclError> {
    let blur_filtered_height_map_buffer = GpuBuffer::new_storage_with_data(
        &device,
        &vec![
            f32::MIN;
            ((map_layout.width_meters() / map_layout.cell_size).ceil() as usize)
                * ((map_layout.height_meters() / map_layout.cell_size).ceil() as usize)
        ],
        Some("filtered_height_map"),
    );
    let (filter_layout, filter_bind_group) = BindGroupLayoutBuilder::new(None)
        .with_entry(
            0,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            input_height_map_buffer.as_entire_binding(),
        )
        .with_entry(
            1,
            ShaderStages::COMPUTE,
            wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            blur_filtered_height_map_buffer.as_entire_binding(),
        )
        .build(&device, "height_map_filter_bind_group".to_string());
    let (filter_shader, filter_constants) = match blur_filter_options {
        BlurFilterOptions::Bilateral(opts) => {
            let constants = HashMap::from([
                (
                    "MAP_WIDTH",
                    (map_layout.width_meters() / map_layout.cell_size).ceil() as f64,
                ),
                (
                    "MAP_HEIGHT",
                    (map_layout.height_meters() / map_layout.cell_size).ceil() as f64,
                ),
                ("SIGMA_SPATIAL", opts.sigma_spatial as f64),
                ("SIGMA_RANGE", opts.sigma_range as f64),
                ("KERNEL_RADIUS", opts.kernel_radius as f64),
                ("WORKGROUP_X", workgroup_size_stage2.0 as f64),
                ("WORKGROUP_Y", workgroup_size_stage2.1 as f64),
                ("CELL_SIZE", map_layout.cell_size as f64),
            ]);
            (
                include_wgsl!("../../shaders/bilateral_filter.wgsl"),
                constants,
            )
        }
        BlurFilterOptions::Gaussian(opts) => {
            let constants = HashMap::from([
                (
                    "MAP_WIDTH",
                    (map_layout.width_meters() / map_layout.cell_size).ceil() as f64,
                ),
                (
                    "MAP_HEIGHT",
                    (map_layout.height_meters() / map_layout.cell_size).ceil() as f64,
                ),
                ("SIGMA", opts.sigma as f64),
                ("CELL_SIZE", map_layout.cell_size as f64),
                ("KERNEL_RADIUS", opts.kernel_radius as f64),
                ("WORKGROUP_X", workgroup_size_stage2.0 as f64),
                ("WORKGROUP_Y", workgroup_size_stage2.1 as f64),
            ]);
            (include_wgsl!("../../shaders/gaussian_blur.wgsl"), constants)
        }
    };
    let filter_pipeline = ComputePipelineBuilder::new(device)
        .with_label("blur_filter")
        .with_shader_module(filter_shader)
        .with_bind_group(0, filter_layout, filter_bind_group)
        .with_constants(filter_constants)
        .build()?;
    Ok((blur_filtered_height_map_buffer, filter_pipeline))
}
