use std::collections::HashMap;

use crate::{errors::WgslPclError, wgsl_setup::GpuDevice};
use anyhow::Result;
use wgpu::{
    BindGroup, BindGroupLayout, ComputePipelineDescriptor, PipelineCompilationOptions,
    PipelineLayout, ShaderModuleDescriptor,
};

pub struct ComputePipeline {
    pub pipeline: wgpu::ComputePipeline,
    pub pipeline_layout: PipelineLayout,
    pub bind_groups: Vec<(usize, BindGroupLayout, BindGroup)>,
}

impl ComputePipeline {
    /// Create a new compute pipeline from a shader module descriptor
    fn new(
        device: &GpuDevice,
        shader_module_descriptor: ShaderModuleDescriptor,
        bind_groups: &[(usize, BindGroupLayout, BindGroup)],
        entry_point: Option<&str>,
        label: Option<&str>,
        constants: Option<HashMap<&str, f64>>,
    ) -> Result<Self, WgslPclError> {
        let module = device.device.create_shader_module(shader_module_descriptor);

        let pipeline_layout =
            device
                .device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: Some("compute_pipeline_layout"),
                    bind_group_layouts: bind_groups
                        .iter()
                        .map(|(_, layout, _)| layout)
                        .collect::<Vec<&BindGroupLayout>>()
                        .as_slice(),
                    immediate_size: 0,
                });

        let constants = constants.unwrap_or_default();

        let pipeline = device
            .device
            .create_compute_pipeline(&ComputePipelineDescriptor {
                label,
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point,
                compilation_options: PipelineCompilationOptions {
                    constants: &constants
                        .iter()
                        .map(|(k, v)| (*k, *v))
                        .collect::<Vec<(&str, f64)>>(),
                    ..Default::default()
                },
                cache: Default::default(),
            });

        Ok(Self {
            pipeline,
            pipeline_layout,
            bind_groups: bind_groups.to_vec(),
        })
    }

    /// Execute the compute pipeline with the given bind groups and workgroup counts
    pub fn execute(&self, device: &GpuDevice, workgroups: (u32, u32, u32)) {
        let mut encoder = device
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("compute_encoder"),
            });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("compute_pass"),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(&self.pipeline);

            for (_, (bind_group_index, _, bind_group)) in self.bind_groups.iter().enumerate() {
                compute_pass.set_bind_group(*bind_group_index as u32, bind_group, &[]);
            }

            compute_pass.dispatch_workgroups(workgroups.0, workgroups.1, workgroups.2);
        }

        device.queue.submit(Some(encoder.finish()));
    }

    /// Execute the compute pipeline and wait for completion
    pub fn execute_blocking(
        &self,
        device: &GpuDevice,
        bind_groups: &[&BindGroup],
        workgroups: (u32, u32, u32),
    ) {
        let mut encoder = device
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("compute_encoder_blocking"),
            });

        {
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some("compute_pass_blocking"),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(&self.pipeline);

            for (index, bind_group) in bind_groups.iter().enumerate() {
                compute_pass.set_bind_group(index as u32, *bind_group, &[]);
            }

            compute_pass.dispatch_workgroups(workgroups.0, workgroups.1, workgroups.2);
        }

        device.queue.submit(Some(encoder.finish()));
        let _ = device.device.poll(wgpu::PollType::wait_indefinitely());
    }
}

/// Builder for creating compute pipelines with a fluent API
pub struct ComputePipelineBuilder<'a> {
    device: &'a GpuDevice,
    shader_module: Option<ShaderModuleDescriptor<'a>>,
    bind_groups: Vec<(usize, BindGroupLayout, BindGroup)>,
    entry_point: Option<&'a str>,
    label: Option<&'a str>,
    constants: Option<HashMap<&'a str, f64>>,
}

impl<'a> ComputePipelineBuilder<'a> {
    pub fn new(device: &'a GpuDevice) -> Self {
        Self {
            device,
            shader_module: None,
            bind_groups: Vec::new(),
            entry_point: None,
            label: None,
            constants: None,
        }
    }

    pub fn with_constants(mut self, constants: HashMap<&'a str, f64>) -> Self {
        self.constants = Some(constants);
        self
    }

    pub fn with_constant(mut self, name: &'a str, value: f64) -> Self {
        if self.constants.is_none() {
            self.constants = Some(HashMap::new());
        }
        if let Some(ref mut consts) = self.constants {
            consts.insert(name, value);
        }
        self
    }

    pub fn with_shader_module(mut self, shader_module: ShaderModuleDescriptor<'a>) -> Self {
        self.shader_module = Some(shader_module);
        self
    }

    pub fn with_bind_group(
        mut self,
        index: usize,
        layout: BindGroupLayout,
        grp: BindGroup,
    ) -> Self {
        self.bind_groups.push((index, layout, grp));
        self
    }

    /// Add multiple bind groups at once
    /// tuple format: (bind group index, layout, bind group)
    pub fn with_bind_group_layouts(
        mut self,
        grps: Vec<(usize, BindGroupLayout, BindGroup)>,
    ) -> Self {
        self.bind_groups.extend(grps);
        self
    }

    pub fn with_entry_point(mut self, entry_point: &'a str) -> Self {
        self.entry_point = Some(entry_point);
        self
    }

    pub fn with_label(mut self, label: &'a str) -> Self {
        self.label = Some(label);
        self
    }

    pub fn build(self) -> Result<ComputePipeline, WgslPclError> {
        let shader_module = self.shader_module.ok_or_else(|| {
            WgslPclError::CompilationError("No shader module provided".to_string())
        })?;

        ComputePipeline::new(
            self.device,
            shader_module,
            &self.bind_groups,
            self.entry_point,
            self.label,
            self.constants,
        )
    }
}

/// Represents a single stage in a multi-stage compute pipeline
pub struct PipelineStage {
    pub pipeline: ComputePipeline,
    pub workgroups: (u32, u32, u32),
    pub label: Option<String>,
}

impl PipelineStage {
    /// you should provide workgroups before calling execute blocking also
    pub fn new(pipeline: ComputePipeline, workgroups: (u32, u32, u32)) -> Self {
        Self {
            pipeline,
            workgroups,
            label: None,
        }
    }

    pub fn with_label(mut self, label: String) -> Self {
        self.label = Some(label);
        self
    }

    pub fn set_workgroup(&mut self, workgroups: (u32, u32, u32)) {
        self.workgroups = workgroups;
    }
}

/// A multi-stage compute pipeline that chains multiple shaders together
/// All data stays on the GPU between stages for maximum performance
pub struct ComputePipelineChain {
    stages: Vec<PipelineStage>,
}

impl ComputePipelineChain {
    pub fn new(stages: Vec<PipelineStage>) -> Self {
        Self { stages }
    }

    /// Execute all stages in sequence within a single command encoder
    /// This keeps all data on the GPU and provides optimal performance
    pub fn execute(
        &mut self,
        device: &GpuDevice,
        workgroups: Vec<(u32, u32, u32)>,
    ) -> Result<(), WgslPclError> {
        if workgroups.len() != self.stages.len() {
            return Err(WgslPclError::WorkgroupError(format!(
                "Expected {} workgroup tuples, but got {}",
                self.stages.len(),
                workgroups.len()
            )));
        }
        let mut encoder = device
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("multi_stage_compute_encoder"),
            });

        for (stage_idx, (stage, workgroup)) in self.stages.iter_mut().zip(workgroups).enumerate() {
            let default_label = format!("compute_pass_stage_{}", stage_idx);
            stage.set_workgroup(workgroup);

            let label = stage
                .label
                .as_ref()
                .map(|s| s.as_str())
                .unwrap_or(&default_label);
            let mut compute_pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor {
                label: Some(label),
                timestamp_writes: None,
            });

            compute_pass.set_pipeline(&stage.pipeline.pipeline);

            for (index, (_, _layout, bind_group)) in stage.pipeline.bind_groups.iter().enumerate() {
                compute_pass.set_bind_group(index as u32, bind_group, &[]);
            }

            compute_pass.dispatch_workgroups(
                stage.workgroups.0,
                stage.workgroups.1,
                stage.workgroups.2,
            );
        }

        device.queue.submit(Some(encoder.finish()));
        Ok(())
    }

    /// Execute all stages and wait for completion
    pub fn execute_blocking(
        &mut self,
        device: &GpuDevice,
        workgroups: Vec<(u32, u32, u32)>,
    ) -> Result<(), WgslPclError> {
        self.execute(device, workgroups)?;
        let _ = device.device.poll(wgpu::PollType::wait_indefinitely());
        Ok(())
    }

    /// Get the number of stages in this pipeline chain
    pub fn stage_count(&self) -> usize {
        self.stages.len()
    }
}

/// Builder for creating multi-stage compute pipelines
pub struct ComputePipelineChainBuilder {
    stages: Vec<PipelineStage>,
}

impl ComputePipelineChainBuilder {
    pub fn new() -> Self {
        Self { stages: Vec::new() }
    }

    /// Add a stage to the pipeline chain
    pub fn add_stage(mut self, stage: PipelineStage) -> Self {
        self.stages.push(stage);
        self
    }

    /// Add a stage with all parameters
    pub fn add_stage_with(
        mut self,
        pipeline: ComputePipeline,
        workgroups: (u32, u32, u32),
        label: Option<String>,
    ) -> Self {
        let mut stage = PipelineStage::new(pipeline, workgroups);
        if let Some(l) = label {
            stage = stage.with_label(l);
        }
        self.stages.push(stage);
        self
    }

    /// Build the pipeline chain
    pub fn build(self) -> Result<ComputePipelineChain, WgslPclError> {
        if self.stages.is_empty() {
            return Err(WgslPclError::CompilationError(
                "Pipeline chain must have at least one stage".to_string(),
            ));
        }
        Ok(ComputePipelineChain::new(self.stages))
    }
}

impl Default for ComputePipelineChainBuilder {
    fn default() -> Self {
        Self::new()
    }
}

/// Creates a vector of ShaderModuleDescriptors from WGSL file paths.
#[macro_export]
macro_rules! shader_module_descriptors_from_wgsl {
    ($( $path:expr ),*) => {
        {
            use wgpu::{ShaderModuleDescriptor, include_wgsl};

            let mut descriptors: Vec<ShaderModuleDescriptor> = Vec::new();
            $(
                descriptors.push(include_wgsl!($path));
            )*
            descriptors
        }
    };
}
