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
    shader_pipeline::{
        ComputePipelineBuilder, ComputePipelineChain, ComputePipelineChainBuilder, PipelineStage,
    },
    wgsl_setup::GpuDevice,
};

pub struct BilateralOptions {
    pub sigma_spatial: u32,
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

pub struct DepthToPclAndHeightPipeline {
    pipeline: ComputePipelineChain,
    depth_image_dimensions_px: (u32, u32),
    depth_scale_buffer: GpuBuffer,
    input_img_buffer: GpuBuffer,
    output_pcl_buffer: GpuBuffer,
    _output_height_map_buffer: GpuBuffer,
    _output_filtered_height_map_buffer: GpuBuffer,
    _output_outlier_filtered_height_map_buffer: GpuBuffer,
    output_gradient_map_buffer: GpuBuffer,
    camera_transform_buffer: GpuBuffer,
    workgroup_size_stage1: (u32, u32, u32),
    workgroup_size_stage2: (u32, u32, u32),
    pub map_layout: map_layout::MapLayout,
}

impl DepthToPclAndHeightPipeline {
    pub fn new(
        max_depth: f32,
        depth_image_dimensions: (u32, u32),
        focal_length_px: f32,
        principal_point_px: (f32, f32),
        device: &GpuDevice,
        workgroup_size_stage1: (u32, u32),
        workgroup_size_stage2: (u32, u32),
        map_layout: map_layout::MapLayout,
        blur_filter_options: BlurFilterOptions,
        outlier_filter_kernel_radius: u32,
        outlier_filter_std_dev_threshold: f32,
        gradient_kernel_radius: u32,
        min_depth: f32,
    ) -> Result<Self, WgslPclError> {
        // Create buffers for depth -> pcl -> height map shader
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

        // define layout and bind groups for depth -> pcl -> height map shader
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

        // define layout and bind groups for camera transform and depth scale
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

        // overridable constants for depth -> pcl -> height map shader
        let constants: HashMap<&str, f64> = HashMap::from([
            ("IMAGE_WIDTH", depth_image_dimensions.0 as f64),
            ("IMAGE_HEIGHT", depth_image_dimensions.1 as f64),
            ("FOCAL_LENGTH_PX", focal_length_px as f64),
            ("PRINCIPAL_POINT_PX_X", principal_point_px.0 as f64),
            ("PRINCIPAL_POINT_PX_Y", principal_point_px.1 as f64),
            ("MAX_DEPTH", max_depth as f64),
            ("MIN_DEPTH", min_depth as f64),
            ("WORKGROUP_X", workgroup_size_stage1.0 as f64),
            ("WORKGROUP_Y", workgroup_size_stage1.1 as f64),
        ]);

        let pipeline = ComputePipelineBuilder::new(device)
            .with_constants(constants)
            .with_bind_group(0, layout_1, bind_group_1)
            .with_bind_group(1, layout_2, bind_group_2)
            .with_shader_module(include_wgsl!("../../shaders/depth_to_pcl_and_height.wgsl"))
            .with_label("depth_to_pcl_pipeline")
            .build()?;

        // Create buffers and pipeline for blur filter (bilateral or gaussian)
        let filtered_height_map_buffer = GpuBuffer::new_storage_with_data(
            &device,
            &vec![
                f32::MIN;
                ((map_layout.width_meters() / map_layout.cell_size).ceil() as usize)
                    * ((map_layout.height_meters() / map_layout.cell_size).ceil() as usize)
            ],
            Some("filtered_height_map"),
        );

        // define layout and bind groups for blur filter shader
        let (filter_layout, filter_bind_group) = BindGroupLayoutBuilder::new(None)
            .with_entry(
                0,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                height_map_buffer.as_entire_binding(),
            )
            .with_entry(
                1,
                ShaderStages::COMPUTE,
                wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: false },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                filtered_height_map_buffer.as_entire_binding(),
            )
            .build(&device, "height_map_filter_bind_group".to_string());

        // Choose shader and constants based on filter type
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

        // Outlier filter
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
            ("KERNEL_RADIUS", outlier_filter_kernel_radius as f64),
            ("OUTLIER_THRESHOLD", outlier_filter_std_dev_threshold as f64),
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
                filtered_height_map_buffer.as_entire_binding(),
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
            .build(&device, "outlier_filter_bind_group".to_string());

        let outlier_filter_pipeline = ComputePipelineBuilder::new(device)
            .with_label("outlier_filter")
            .with_shader_module(include_wgsl!("../../shaders/outlier_removal.wgsl"))
            .with_bind_group(0, outlier_filter_layout, outlier_filter_bind_group)
            .with_constants(outlier_filter_constants)
            .build()?;

        // Gradient mapper
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
                outlier_filtered_height_map_buffer.as_entire_binding(),
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

        let combined_pipeline = ComputePipelineChainBuilder::new()
            .add_stage(PipelineStage::new(
                pipeline,
                (workgroup_size_stage1.0, workgroup_size_stage1.1, 1),
            ))
            .add_stage(PipelineStage::new(
                filter_pipeline,
                (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            ))
            .add_stage(PipelineStage::new(
                outlier_filter_pipeline,
                (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            ))
            .add_stage(PipelineStage::new(
                gradient_pipeline,
                (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            ))
            .build()?;

        Ok(Self {
            pipeline: combined_pipeline,
            depth_image_dimensions_px: depth_image_dimensions,
            _output_height_map_buffer: height_map_buffer,
            input_img_buffer,
            output_pcl_buffer,
            camera_transform_buffer,
            depth_scale_buffer,
            map_layout,
            _output_filtered_height_map_buffer: filtered_height_map_buffer,
            _output_outlier_filtered_height_map_buffer: outlier_filtered_height_map_buffer,
            output_gradient_map_buffer: gradient_map_buffer,
            workgroup_size_stage1: (workgroup_size_stage1.0, workgroup_size_stage1.1, 1),
            workgroup_size_stage2: (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
        })
    }

    /// returns (point_cloud, gradient_map)
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

        let workgroups_pcl = (
            (self.depth_image_dimensions_px.0 + self.workgroup_size_stage1.0 - 1)
                / self.workgroup_size_stage1.0,
            (self.depth_image_dimensions_px.1 + self.workgroup_size_stage1.1 - 1)
                / self.workgroup_size_stage1.1,
            1,
        );
        let workgroups_filter = (
            ((self.map_layout.width_meters() / self.map_layout.cell_size).ceil() as u32
                + self.workgroup_size_stage2.0
                - 1)
                / self.workgroup_size_stage2.0,
            ((self.map_layout.height_meters() / self.map_layout.cell_size).ceil() as u32
                + self.workgroup_size_stage2.1
                - 1)
                / self.workgroup_size_stage2.1,
            1,
        );
        let workgroups_outlier_filter = (
            ((self.map_layout.width_meters() / self.map_layout.cell_size).ceil() as u32 + 8 - 1)
                / 8,
            ((self.map_layout.height_meters() / self.map_layout.cell_size).ceil() as u32 + 8 - 1)
                / 8,
            1,
        );
        let workgroups_gradient = workgroups_filter;

        let workgroups = vec![
            workgroups_pcl,
            workgroups_filter,
            workgroups_outlier_filter,
            workgroups_gradient,
        ];

        self.pipeline.execute_blocking(device, workgroups)?;
        let pcl_data: Vec<AlignedVec4<f32>> = self.output_pcl_buffer.read_data_blocking(device)?;
        let gradient_data: Vec<f32> = self.output_gradient_map_buffer.read_data_blocking(device)?;

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
            gradient_data,
        ))
    }

    pub fn get_height_map(&self, device: &GpuDevice) -> anyhow::Result<Vec<f32>> {
        let height_map_data: Vec<f32> = self
            ._output_outlier_filtered_height_map_buffer
            .read_data_blocking(device)?;
        Ok(height_map_data)
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
            3.0,                    // max_depth
            (640, 480),             // dimensions
            383.0,                  // focal_length
            (317.44882, 245.91605), // principal point
            device,
            (workgroup_size.0, workgroup_size.1), // stage1 workgroup
            (8, 8),                               // stage2 workgroup (fixed for now)
            map_layout::MapLayout::new(10.0, -10.0, 10.0, -10.0, 0.05), // map layout
            BlurFilterOptions::Gaussian(GaussianOptions {
                kernel_radius: 9,
                sigma: 3.0,
            }),
            2,   // outlier_filter_kernel_radius
            2.0, // outlier_filter_std_dev_threshold
            1,   // gradient_kernel_radius
            2.0, // min_depth
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
