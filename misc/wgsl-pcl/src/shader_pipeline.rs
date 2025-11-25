use crate::{errors::WgslPclError, wgsl_setup::GpuDevice};
use anyhow::Result;
use wgpu::{
    BindGroupLayout, ComputePipelineDescriptor, PipelineLayout, PipelineLayoutDescriptor,
    ShaderModuleDescriptor, include_wgsl,
};

pub struct ComputePipeline {
    device: GpuDevice,
}

impl ComputePipeline {
    pub fn from_shader_modules(
        &self,
        shader_module_descriptors: Vec<ShaderModuleDescriptor>,
        bind_group_layouts: Vec<&BindGroupLayout>,
    ) -> Result<Self, WgslPclError> {
        let mut modules = Vec::new();
        for descriptor in shader_module_descriptors {
            let module = self.device.device.create_shader_module(descriptor);
            modules.push(module);
        }
        let pipeline_layout =
            self.device
                .device
                .create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                    label: None,
                    bind_group_layouts: &bind_group_layouts,
                    push_constant_ranges: &[],
                });
        for module in modules {
            let compute_pipeline_descriptor = ComputePipelineDescriptor {
                label: None,
                layout: Some(&pipeline_layout),
                module: &module,
                entry_point: None,
                compilation_options: Default::default(),
                cache: Default::default(),
            };
        }
        todo!()
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
