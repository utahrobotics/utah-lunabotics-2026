use core::f32;
use std::collections::HashMap;

use nalgebra::Vector3;
use wgpu::{ShaderStages, include_wgsl};

use crate::{
    benchmark::BenchmarkableComputePipeline,
    errors::WgslPclError,
    gpu_types::{AlignedMatrix4, AlignedVec4, GpuType},
    map_layout,
    mem_layouts::{BindGroupLayoutBuilder, GpuBuffer},
    shader_pipeline::{ComputePipeline, ComputePipelineBuilder},
    wgsl_setup::GpuDevice,
};

pub struct DepthToPclAndHeightPipeline {
    pipeline: ComputePipeline,
    /// Max depth in meters
    depth_image_dimensions_px: (u32, u32),

    depth_scale_buffer: GpuBuffer,
    bind_group_1: wgpu::BindGroup,
    bind_group_2: wgpu::BindGroup,
    input_img_buffer: GpuBuffer,
    output_pcl_buffer: GpuBuffer,
    output_height_map_buffer: GpuBuffer,
    camera_transform_buffer: GpuBuffer,
    workgroup_size: (u32, u32),
    pub map_layout: map_layout::MapLayout,
}

impl DepthToPclAndHeightPipeline {
    pub fn new(
        max_depth: f32,
        depth_image_dimensions: (u32, u32),
        focal_length_px: f32,
        principal_point_px: (f32, f32),
        device: &GpuDevice,
        workgroup_size: (u32, u32),
        map_layout: map_layout::MapLayout,
    ) -> Result<Self, WgslPclError> {
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
        let map_layout_buffer =
            GpuBuffer::new_uniform_with_data(&device, &map_layout, Some("map_layout_buffer"));
        let height_map_buffer = GpuBuffer::new_storage_with_data(
            &device,
            &vec![
                f32::MIN;
                ((map_layout.width_meters() / map_layout.cell_size).ceil() as usize)
                    * ((map_layout.height_meters() / map_layout.cell_size).ceil() as usize)
            ],
            Some("height_map"),
        );
        let (layout_1, bind_group_1) = BindGroupLayoutBuilder::new(None)
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
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                height_map_buffer.as_entire_binding(),
            )
            .build(&device, "bind_group_0".to_string());

        let (layout_2, bind_group_2) = BindGroupLayoutBuilder::new(None)
            .with_entry(
                0,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                camera_transform_buffer.as_entire_binding(),
            )
            .with_entry(
                1,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                depth_scale_buffer.as_entire_binding(),
            )
            .with_entry(
                2,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                map_layout_buffer.as_entire_binding(),
            )
            .build(&device, "depth_to_pcl_bind_group".to_string());

        let constants: HashMap<&str, f64> = HashMap::from([
            ("IMAGE_WIDTH", depth_image_dimensions.0 as f64),
            ("IMAGE_HEIGHT", depth_image_dimensions.1 as f64),
            ("FOCAL_LENGTH_PX", focal_length_px as f64),
            ("PRINCIPAL_POINT_PX_X", principal_point_px.0 as f64),
            ("PRINCIPAL_POINT_PX_Y", principal_point_px.1 as f64),
            ("MAX_DEPTH", max_depth as f64),
            ("WORKGROUP_X", workgroup_size.0 as f64),
            ("WORKGROUP_Y", workgroup_size.1 as f64),
        ]);
        let pipeline = ComputePipelineBuilder::new(device)
            .with_constants(constants)
            .with_bind_group_layout(&layout_1)
            .with_bind_group_layout(&layout_2)
            .with_shader_module(include_wgsl!("../../shaders/depth_to_pcl_and_height.wgsl"))
            .with_label("depth_to_pcl_pipeline")
            .build()?;
        Ok(Self {
            pipeline,
            depth_image_dimensions_px: depth_image_dimensions,
            bind_group_1,
            bind_group_2,
            output_height_map_buffer: height_map_buffer,
            input_img_buffer,
            output_pcl_buffer,
            camera_transform_buffer,
            depth_scale_buffer,
            workgroup_size,
            map_layout,
        })
    }

    /// returns (point_cloud, height_map)
    pub fn process(
        &mut self,
        depths: &[u16],
        device: &GpuDevice,
        camera_transform: AlignedMatrix4<f32>,
        depth_scale: f32,
    ) -> anyhow::Result<(Vec<Vector3<f32>>, Vec<f32>)> {
        self.camera_transform_buffer
            .write_data(device, camera_transform.to_bytes(), 0);
        self.input_img_buffer.write_data(device, depths, 0);
        self.depth_scale_buffer
            .write_data(device, depth_scale.to_bytes(), 0);

        self.pipeline.execute_blocking(
            device,
            &[&self.bind_group_1, &self.bind_group_2],
            (
                (self.depth_image_dimensions_px.0 + self.workgroup_size.0 - 1)
                    / self.workgroup_size.0,
                (self.depth_image_dimensions_px.1 + self.workgroup_size.1 - 1)
                    / self.workgroup_size.1,
                1,
            ),
        );
        let pcl_data: Vec<AlignedVec4<f32>> = self.output_pcl_buffer.read_data_blocking(device)?;
        let heightmap_data: Vec<f32> = self.output_height_map_buffer.read_data_blocking(device)?;

        Ok((
            pcl_data
                .iter()
                .filter_map(|v| {
                    if v.w != 0.0 {
                        Some(Vector3::new(v.x, v.y, v.z))
                    } else {
                        None
                    }
                })
                .collect(),
            heightmap_data,
        ))
    }
}

pub struct DepthToPclBenchmarkInput {
    pub depths: Vec<u16>,
    pub camera_transform: AlignedMatrix4<f32>,
    pub depth_scale: f32,
    pub max_depth: f32,
    pub depth_image_dimensions: (u32, u32),
    pub focal_length_px: f32,
    pub principal_point_px: (f32, f32),
}

impl BenchmarkableComputePipeline for DepthToPclAndHeightPipeline {
    type Input = DepthToPclBenchmarkInput;
    type Output = (Vec<Vector3<f32>>, Vec<f32>);

    fn create_with_workgroup(
        device: &GpuDevice,
        workgroup_size: (u32, u32, u32),
    ) -> anyhow::Result<Self>
    where
        Self: Sized,
    {
        Ok(Self::new(
            3.0,
            (640, 480),
            383.0,
            (320.0, 240.0),
            device,
            (workgroup_size.0, workgroup_size.1),
            map_layout::MapLayout::new(10.0, -10.0, 10.0, -10.0, 0.1),
        )?)
    }

    fn execute_once(
        &mut self,
        device: &GpuDevice,
        input: &Self::Input,
    ) -> anyhow::Result<Self::Output> {
        self.process(
            &input.depths,
            device,
            input.camera_transform,
            input.depth_scale,
        )
    }

    fn operation_count(&self) -> u64 {
        (self.depth_image_dimensions_px.0 * self.depth_image_dimensions_px.1) as u64
    }
}
