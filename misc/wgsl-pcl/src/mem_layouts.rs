use crate::wgsl_setup::GpuDevice;
use std::num::NonZeroU32;
use wgpu::{BindGroup, BindingResource, BindingType, ShaderStages};

pub struct PushConstantRangesBuilder {
    ranges: Vec<wgpu::PushConstantRange>,
}

pub struct BindGroupLayoutBuilder<'a> {
    entries: Vec<wgpu::BindGroupLayoutEntry>,
    resources: Vec<BindingResource<'a>>,
    total_count: Option<NonZeroU32>,
}

impl<'a> BindGroupLayoutBuilder<'a> {
    pub fn new(total_count: Option<NonZeroU32>) -> Self {
        Self {
            entries: Vec::new(),
            resources: Vec::new(),
            total_count,
        }
    }

    pub fn with_entry(
        mut self,
        binding_index: u32,
        visibility: ShaderStages,
        binding_type: BindingType,
        buffer: BindingResource<'a>,
    ) -> Self {
        self.entries.push(wgpu::BindGroupLayoutEntry {
            binding: binding_index,
            visibility,
            ty: binding_type,
            count: self.total_count,
        });
        self.resources.push(buffer);
        self
    }

    pub fn build(&self, device: &GpuDevice, label: String) -> wgpu::BindGroupLayout {
        let main_descriptor = wgpu::BindGroupLayoutDescriptor {
            label: Some(&label),
            entries: &self.entries,
        };
        device.device.create_bind_group_layout(&main_descriptor)
    }

    /// Creates bind groups based on the layout entries and provided buffers
    pub fn to_bind_group(
        &self,
        device: &GpuDevice,
        label: String,
        buffers: Vec<BindingResource>,
    ) -> BindGroup {
        let layout = self.build(device, label);
        let grp = device.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &self
                .entries
                .iter()
                .zip(buffers)
                .map(|(layout_entry, resource)| wgpu::BindGroupEntry {
                    binding: layout_entry.binding,
                    resource,
                })
                .collect::<Vec<wgpu::BindGroupEntry<'_>>>(),
        });
        grp
    }
}
