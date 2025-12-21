use core::f32;
use std::collections::HashMap;

use nalgebra::Vector3;
use wgpu::{ShaderStages, include_wgsl};

use crate::{
    errors::WgslPclError,
    gpu_types::{AlignedMatrix4, AlignedVec4, GpuType},
    map_layout,
    mem_layouts::{BindGroupLayoutBuilder, GpuBuffer},
    pipelines::{
        expander::new_obstacle_expander_pipeline,
        filters::{
            BlurFilterOptions, OutlierFilterOptions, new_blur_pipeline,
            new_outlier_removal_pipeline,
        },
        gradient::new_gradient_pipeline,
    },
    shader_pipeline::{
        ComputePipelineBuilder, ComputePipelineChain, ComputePipelineChainBuilder, PipelineStage,
    },
    wgsl_setup::GpuDevice,
};

pub struct ObstacleExpanderOptions {
    pub expansion_radius_meters: f32,
    pub obstacle_gradient_threshold: f32,
}

/// When enabled, only cells that will be affected by the new frame are cleared
#[derive(Clone, Copy)]
pub struct ClearAffectedCellsOptions {
    // sometimes only clearing cells further away is desired
    // this minimizes unnecessary clearing of nearby cells that are likely to be updated again
    pub min_distance_to_clear: f32,
}

pub struct DepthToPclAndHeightPipeline {
    pipeline: ComputePipelineChain,
    depth_image_dimensions_px: (u32, u32),
    depth_scale_buffer: GpuBuffer,
    input_img_buffer: GpuBuffer,
    output_pcl_buffer: GpuBuffer,
    output_height_map_buffer: GpuBuffer,
    blur_filtered_height_map_buffer: GpuBuffer,
    output_outlier_filtered_height_map_buffer: GpuBuffer,
    output_gradient_map_buffer: GpuBuffer,
    output_expanded_gradient_map_buffer: GpuBuffer,
    camera_transform_buffer: GpuBuffer,
    workgroup_size_stage1: (u32, u32, u32),
    workgroup_size_stage2: (u32, u32, u32),
    pub map_layout: map_layout::MapLayout,
    map_layout_buffer: GpuBuffer,
    map_height_buffer: GpuBuffer,
    map_width_buffer: GpuBuffer,
    /// Whether the clear affected cells stage is enabled
    clear_affected_cells_enabled: bool,
}

impl DepthToPclAndHeightPipeline {
    /// stage 1 workgroup size is for depth -> pcl + height map
    /// stage 2 workgroup size is for filtering and gradient mapping
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
        outlier_filter_options: OutlierFilterOptions,
        obstacle_expander_options: ObstacleExpanderOptions,
        gradient_kernel_radius: u32,
        min_depth: f32,
        clear_affected_cells: Option<ClearAffectedCellsOptions>,
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

        // Create clear affected cells pipeline if enabled
        let clear_stage = if let Some(clear_options) = clear_affected_cells {
            let clear_constants: HashMap<&str, f64> = HashMap::from([
                ("IMAGE_WIDTH", depth_image_dimensions.0 as f64),
                ("IMAGE_HEIGHT", depth_image_dimensions.1 as f64),
                ("FOCAL_LENGTH_PX", focal_length_px as f64),
                ("PRINCIPAL_POINT_PX_X", principal_point_px.0 as f64),
                ("PRINCIPAL_POINT_PX_Y", principal_point_px.1 as f64),
                ("MAX_DEPTH", max_depth as f64),
                ("MIN_DEPTH", min_depth as f64),
                ("WORKGROUP_X", workgroup_size_stage1.0 as f64),
                ("WORKGROUP_Y", workgroup_size_stage1.1 as f64),
                (
                    "MIN_DISTANCE_TO_CLEAR",
                    clear_options.min_distance_to_clear as f64,
                ),
            ]);

            let (img_buffers_layout, img_buffers_bindgrp) = BindGroupLayoutBuilder::new(None)
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
                    height_map_buffer.as_entire_binding(),
                )
                .build(&device, "clear_affected_bind_group_0".to_string());

            let (clear_map_layout_transforms_layout, clear_map_layout_transforms_bindgrp) =
                BindGroupLayoutBuilder::new(None)
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
                    .build(&device, "clear_affected_bind_group_1".to_string());

            let clear_pipeline = ComputePipelineBuilder::new(device)
                .with_constants(clear_constants)
                .with_bind_group(
                    0,
                    clear_map_layout_transforms_layout,
                    clear_map_layout_transforms_bindgrp,
                )
                .with_bind_group(1, img_buffers_layout, img_buffers_bindgrp)
                .with_shader_module(include_wgsl!("../../shaders/clear_affected_cells.wgsl"))
                .with_label("clear_affected_cells_pipeline")
                .build()?;

            Some(clear_pipeline)
        } else {
            None
        };

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
        let (transforms_map_layout, transforms_map_layout_bindgrp) =
            BindGroupLayoutBuilder::new(None)
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
            ("MIN_DEPTH", min_depth as f64),
            ("WORKGROUP_X", workgroup_size_stage1.0 as f64),
            ("WORKGROUP_Y", workgroup_size_stage1.1 as f64),
        ]);

        let pipeline = ComputePipelineBuilder::new(device)
            .with_constants(constants)
            .with_bind_group(0, transforms_map_layout, transforms_map_layout_bindgrp)
            .with_bind_group(1, layout_1, bind_group_1)
            .with_shader_module(include_wgsl!("../../shaders/depth_to_pcl_and_height.wgsl"))
            .with_label("depth_to_pcl_pipeline")
            .build()?;

        let map_height_buffer = GpuBuffer::new_uniform_with_data(
            &device,
            &((map_layout.height_meters() / map_layout.cell_size).ceil() as u32),
            Some("map_layout_buffer"),
        );
        let map_width_buffer = GpuBuffer::new_uniform_with_data(
            &device,
            &((map_layout.width_meters() / map_layout.cell_size).ceil() as u32),
            Some("map_layout_buffer"),
        );

        let (outlier_filtered_height_map_buffer, outlier_filter_pipeline) =
            new_outlier_removal_pipeline(
                device,
                map_layout,
                outlier_filter_options,
                &height_map_buffer,
                &map_width_buffer,
                &map_height_buffer,
            )?;

        let (blur_filtered_height_map_buffer, filter_pipeline) = new_blur_pipeline(
            device,
            workgroup_size_stage2,
            map_layout,
            blur_filter_options,
            &outlier_filtered_height_map_buffer,
            &map_width_buffer,
            &map_height_buffer,
        )?;

        let (gradient_map_buffer, gradient_pipeline) = new_gradient_pipeline(
            device,
            workgroup_size_stage2,
            map_layout,
            gradient_kernel_radius,
            &blur_filtered_height_map_buffer,
            &map_width_buffer,
            &map_height_buffer,
        )?;

        let (obstacle_expander_output_buffer, obstacle_expander_pipeline) =
            new_obstacle_expander_pipeline(
                device,
                map_layout,
                obstacle_expander_options,
                &gradient_map_buffer,
                &map_width_buffer,
                &map_height_buffer,
            )?;

        let mut chain_builder = ComputePipelineChainBuilder::new();

        // Add clear stage first if enabled
        if let Some(clear_pipeline) = clear_stage {
            chain_builder = chain_builder.add_stage(PipelineStage::new(
                clear_pipeline,
                (workgroup_size_stage1.0, workgroup_size_stage1.1, 1),
            ));
        }

        let combined_pipeline = chain_builder
            .add_stage(PipelineStage::new(
                pipeline,
                (workgroup_size_stage1.0, workgroup_size_stage1.1, 1),
            ))
            .add_stage(PipelineStage::new(
                outlier_filter_pipeline,
                (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            ))
            .add_stage(PipelineStage::new(
                filter_pipeline,
                (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            ))
            .add_stage(PipelineStage::new(
                gradient_pipeline,
                (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            ))
            .add_stage(PipelineStage::new(
                obstacle_expander_pipeline,
                (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            ))
            .build()?;

        Ok(Self {
            pipeline: combined_pipeline,
            depth_image_dimensions_px: depth_image_dimensions,
            output_height_map_buffer: height_map_buffer,
            input_img_buffer,
            output_pcl_buffer,
            camera_transform_buffer,
            depth_scale_buffer,
            map_layout,
            blur_filtered_height_map_buffer,
            output_outlier_filtered_height_map_buffer: outlier_filtered_height_map_buffer,
            output_gradient_map_buffer: gradient_map_buffer,
            output_expanded_gradient_map_buffer: obstacle_expander_output_buffer,
            workgroup_size_stage1: (workgroup_size_stage1.0, workgroup_size_stage1.1, 1),
            workgroup_size_stage2: (workgroup_size_stage2.0, workgroup_size_stage2.1, 1),
            clear_affected_cells_enabled: clear_affected_cells.is_some(),
            map_width_buffer,
            map_height_buffer,
            map_layout_buffer,
        })
    }

    /// returns (point_cloud, gradient_map)
    pub fn process(
        &mut self,
        depths: &[u16],
        device: &GpuDevice,
        camera_transform: AlignedMatrix4<f32>,
        depth_scale: f32,
        // map_layout: &map_layout::MapLayout,
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
        let workgroups_obstacle_expander = (
            ((self.map_layout.width_meters() / self.map_layout.cell_size).ceil() as u32 + 8 - 1)
                / 8,
            ((self.map_layout.height_meters() / self.map_layout.cell_size).ceil() as u32 + 8 - 1)
                / 8,
            1,
        );

        let workgroups = if self.clear_affected_cells_enabled {
            vec![
                workgroups_pcl, // clear stage uses same workgroups as pcl
                workgroups_pcl,
                workgroups_outlier_filter,
                workgroups_filter,
                workgroups_gradient,
                workgroups_obstacle_expander,
            ]
        } else {
            vec![
                workgroups_pcl,
                workgroups_outlier_filter,
                workgroups_filter,
                workgroups_gradient,
                workgroups_obstacle_expander,
            ]
        };

        self.pipeline.execute_blocking(device, workgroups)?;
        let pcl_data: Vec<AlignedVec4<f32>> = self.output_pcl_buffer.read_data_blocking(device)?;
        let gradient_data: Vec<f32> = self
            .output_expanded_gradient_map_buffer
            .read_data_blocking(device)?;

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

    pub fn get_outlier_filtered_height_map(&self, device: &GpuDevice) -> anyhow::Result<Vec<f32>> {
        let height_map_data: Vec<f32> = self
            .output_outlier_filtered_height_map_buffer
            .read_data_blocking(device)?;
        Ok(height_map_data)
    }

    pub fn get_blur_filtered_height_map(&self, device: &GpuDevice) -> anyhow::Result<Vec<f32>> {
        let height_map_data: Vec<f32> = self
            .blur_filtered_height_map_buffer
            .read_data_blocking(device)?;
        Ok(height_map_data)
    }

    pub fn get_obstacle_expanded_height_map(&self, device: &GpuDevice) -> anyhow::Result<Vec<f32>> {
        let height_map_data: Vec<f32> = self
            .output_expanded_gradient_map_buffer
            .read_data_blocking(device)?;
        Ok(height_map_data)
    }

    pub fn get_raw_height_map(&self, device: &GpuDevice) -> anyhow::Result<Vec<f32>> {
        let height_map_data: Vec<f32> = self.output_height_map_buffer.read_data_blocking(device)?;
        Ok(height_map_data)
    }

    pub fn get_gradient_map(&self, device: &GpuDevice) -> anyhow::Result<Vec<f32>> {
        let gradient_map_data: Vec<f32> =
            self.output_gradient_map_buffer.read_data_blocking(device)?;
        Ok(gradient_map_data)
    }
}
