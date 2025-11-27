use std::collections::HashMap;

use nalgebra::Vector3;
use wgpu::{ShaderStages, include_wgsl};

use crate::{
    gpu_types::{AlignedMatrix4, AlignedVec4, GpuType},
    mem_layouts::{BindGroupLayoutBuilder, GpuBuffer},
    shader_pipeline::{ComputePipeline, ComputePipelineBuilder},
    wgsl_setup::GpuDevice,
};

pub struct DepthToPclPipeline {
    pipeline: ComputePipeline,
    /// Max depth in meters
    depth_image_dimensions_px: (u32, u32),

    depth_scale_buffer: GpuBuffer,
    bind_group: wgpu::BindGroup,
    input_img_buffer: GpuBuffer,
    output_pcl_buffer: GpuBuffer,
    camera_transform_buffer: GpuBuffer,
}

impl DepthToPclPipeline {
    pub fn new(
        max_depth: f32,
        depth_image_dimensions: (u32, u32),
        focal_length_px: f32,
        principal_point_px: (f32, f32),
        device: &GpuDevice,
    ) -> Self {
        let input_img_buffer = GpuBuffer::new_storage(
            &device,
            ((depth_image_dimensions.0 * depth_image_dimensions.1) * size_of::<u16>() as u32)
                as u64,
            Some("input_img_buffer"),
        );
        let output_pcl_buffer = GpuBuffer::new_storage(
            &device,
            ((depth_image_dimensions.0 * depth_image_dimensions.1)
                * size_of::<AlignedVec4<f32>>() as u32) as u64,
            Some("output_pcl_buffer"),
        );
        let camera_transform_buffer = GpuBuffer::new_uniform(
            &device,
            size_of::<AlignedMatrix4<f32>>() as u64,
            Some("camera_transform_buffer"),
        );
        let depth_scale_buffer =
            GpuBuffer::new_uniform(&device, size_of::<f32>() as u64, Some("depth_scale_buffer"));
        let (layout, bind_group) = BindGroupLayoutBuilder::new(None)
            .with_entry(
                0,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                input_img_buffer.as_entire_binding(),
            )
            .with_entry(
                1,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                output_pcl_buffer.as_entire_binding(),
            )
            .with_entry(
                2,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                camera_transform_buffer.as_entire_binding(),
            )
            .with_entry(
                3,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                depth_scale_buffer.as_entire_binding(),
            )
            .build(&device, "depth_to_pcl_bind_group".to_string());

        let constants: HashMap<&str, f64> = HashMap::from([
            ("IMAGE_WIDTH", depth_image_dimensions.0 as f64),
            ("IMAGE_HEIGHT", depth_image_dimensions.1 as f64),
            ("FOCAL_LENGTH_PX", focal_length_px as f64),
            ("PRINCIPAL_POINT_PX_X", principal_point_px.0 as f64),
            ("PRINCIPAL_POINT_PX_Y", principal_point_px.1 as f64),
            ("MAX_DEPTH", max_depth as f64),
        ]);
        let pipeline = ComputePipelineBuilder::new(device)
            .with_constants(constants)
            .with_bind_group_layout(&layout)
            .with_shader_module(include_wgsl!("../../shaders/depth_to_pcl.wgsl"))
            .with_label("depth_to_pcl_pipeline")
            .build()
            .unwrap();
        Self {
            pipeline,
            depth_image_dimensions_px: depth_image_dimensions,

            bind_group,
            input_img_buffer,
            output_pcl_buffer,
            camera_transform_buffer,
            depth_scale_buffer,
        }
    }

    pub fn process(
        &mut self,
        depths: &[u16],
        device: &GpuDevice,
        camera_transform: AlignedMatrix4<f32>,
        depth_scale: f32,
    ) -> anyhow::Result<Vec<Vector3<f32>>> {
        self.camera_transform_buffer
            .write_data(device, camera_transform.to_bytes(), 0);
        self.input_img_buffer.write_data(device, depths, 0);
        self.depth_scale_buffer
            .write_data(device, depth_scale.to_bytes(), 0);

        self.pipeline.execute_blocking(
            device,
            &[&self.bind_group],
            (
                (self.depth_image_dimensions_px.0 + 7) / 8,
                (self.depth_image_dimensions_px.1 + 7) / 8,
                1,
            ),
        );
        let pcl_data: Vec<AlignedVec4<f32>> = self.output_pcl_buffer.read_data_blocking(device)?;

        Ok(pcl_data
            .iter()
            .filter_map(|v| {
                if v.w != 0.0 {
                    Some(Vector3::new(v.x, v.y, v.z))
                } else {
                    None
                }
            })
            .collect())
    }
}
