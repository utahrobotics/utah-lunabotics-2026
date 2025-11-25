use crate::wgsl_setup::GpuDevice;
use std::num::NonZeroU32;
use wgpu::{BindingType, ShaderStages};

pub struct BindGroupLayoutsBuilder {
    layouts: Vec<wgpu::BindGroupLayout>,
}

pub struct PushConstantRangesBuilder {
    ranges: Vec<wgpu::PushConstantRange>,
}

pub struct BindGroupLayoutBuilder {
    entries: Vec<wgpu::BindGroupLayoutEntry>,
    total_count: Option<NonZeroU32>,
}

impl BindGroupLayoutBuilder {
    pub fn new(total_count: Option<NonZeroU32>) -> Self {
        Self {
            entries: Vec::new(),
            total_count,
        }
    }

    pub fn with_entry(
        mut self,
        binding_index: u32,
        visibility: ShaderStages,
        binding_type: BindingType,
    ) -> Self {
        self.entries.push(wgpu::BindGroupLayoutEntry {
            binding: binding_index,
            visibility,
            ty: binding_type,
            count: self.total_count,
        });
        self
    }

    pub fn build(self, device: GpuDevice, label: String) -> wgpu::BindGroupLayout {
        let main_descriptor = wgpu::BindGroupLayoutDescriptor {
            label: Some(&label),
            entries: &self.entries,
        };
        device.device.create_bind_group_layout(&main_descriptor)
    }
}
