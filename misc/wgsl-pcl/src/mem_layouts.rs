use crate::wgsl_setup::GpuDevice;
use anyhow::Result;
use std::num::NonZeroU32;
use wgpu::{BindGroup, BindingResource, BindingType, BufferUsages, ShaderStages};

pub struct BindGroupLayoutBuilder<'a> {
    entries: Vec<wgpu::BindGroupLayoutEntry>,
    resources: Vec<BindingResource<'a>>,
    binding_array_len: Option<NonZeroU32>,
}

impl<'a> BindGroupLayoutBuilder<'a> {
    pub fn new(binding_array_len: Option<NonZeroU32>) -> Self {
        Self {
            entries: Vec::new(),
            resources: Vec::new(),
            binding_array_len,
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
            count: self.binding_array_len,
        });
        self.resources.push(buffer);
        self
    }

    pub fn build(self, device: &GpuDevice, label: String) -> (wgpu::BindGroupLayout, BindGroup) {
        let main_descriptor = wgpu::BindGroupLayoutDescriptor {
            label: Some(&label),
            entries: &self.entries,
        };
        let layout = device.device.create_bind_group_layout(&main_descriptor);
        let grp = device.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some(&format!("{}_bind_group", label)),
            layout: &layout,
            entries: &self
                .entries
                .iter()
                .zip(self.resources.into_iter())
                .map(|(layout_entry, resource)| wgpu::BindGroupEntry {
                    binding: layout_entry.binding,
                    resource: resource,
                })
                .collect::<Vec<wgpu::BindGroupEntry<'_>>>(),
        });
        (layout, grp)
    }
}

/// Helper struct for creating and managing GPU buffers
pub struct GpuBuffer {
    pub buffer: wgpu::Buffer,
    pub size: u64,
}

impl GpuBuffer {
    /// Create a new storage buffer with the given size and usage flags
    pub fn new_storage(device: &GpuDevice, size: u64, label: Option<&str>) -> Self {
        let buffer = device.device.create_buffer(&wgpu::BufferDescriptor {
            label,
            size,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        Self { buffer, size }
    }

    /// Create a new uniform buffer with the given size
    pub fn new_uniform(device: &GpuDevice, size: u64, label: Option<&str>) -> Self {
        let buffer = device.device.create_buffer(&wgpu::BufferDescriptor {
            label,
            size,
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self { buffer, size }
    }

    pub fn new_uniform_with_data<T: bytemuck::Pod>(
        device: &GpuDevice,
        data: &T,
        label: Option<&str>,
    ) -> Self {
        let bytes = bytemuck::bytes_of(data);
        let buffer = device.device.create_buffer(&wgpu::BufferDescriptor {
            label,
            size: bytes.len() as u64,
            usage: BufferUsages::UNIFORM | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        device.queue.write_buffer(&buffer, 0, bytes);
        Self {
            buffer,
            size: bytes.len() as u64,
        }
    }

    /// Create a buffer with custom usage flags
    pub fn new_with_usage(
        device: &GpuDevice,
        size: u64,
        usage: BufferUsages,
        label: Option<&str>,
    ) -> Self {
        let buffer = device.device.create_buffer(&wgpu::BufferDescriptor {
            label,
            size,
            usage,
            mapped_at_creation: false,
        });
        Self { buffer, size }
    }

    /// Create a storage buffer and initialize it with data
    pub fn new_storage_with_data<T: bytemuck::Pod>(
        device: &GpuDevice,
        data: &[T],
        label: Option<&str>,
    ) -> Self {
        let bytes = bytemuck::cast_slice(data);
        let buffer = device.device.create_buffer(&wgpu::BufferDescriptor {
            label,
            size: bytes.len() as u64,
            usage: BufferUsages::STORAGE | BufferUsages::COPY_DST | BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        device.queue.write_buffer(&buffer, 0, bytes);
        Self {
            buffer,
            size: bytes.len() as u64,
        }
    }

    /// Write data to the buffer at the given offset
    pub fn write_data<T: bytemuck::Pod>(&self, device: &GpuDevice, data: &[T], offset: u64) {
        let bytes = bytemuck::cast_slice(data);
        device.queue.write_buffer(&self.buffer, offset, bytes);
    }

    /// Read data from the buffer asynchronously
    pub async fn read_data<T: bytemuck::Pod>(&self, device: &GpuDevice) -> Result<Vec<T>> {
        // Create a staging buffer with MAP_READ usage
        let staging_buffer = device.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("staging_buffer"),
            size: self.size,
            usage: BufferUsages::MAP_READ | BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        // Copy from GPU buffer to staging buffer
        let mut encoder = device
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("copy_encoder"),
            });
        encoder.copy_buffer_to_buffer(&self.buffer, 0, &staging_buffer, 0, self.size);
        device.queue.submit(Some(encoder.finish()));

        // Map the staging buffer and read the data
        let buffer_slice = staging_buffer.slice(..);
        let (sender, receiver) = futures::channel::oneshot::channel();
        buffer_slice.map_async(wgpu::MapMode::Read, move |result| {
            sender.send(result).unwrap();
        });

        let _ = device.device.poll(wgpu::PollType::wait_indefinitely());
        receiver.await??;

        let data = buffer_slice.get_mapped_range();
        let result: Vec<T> = bytemuck::cast_slice(&data).to_vec();
        drop(data);
        staging_buffer.unmap();

        Ok(result)
    }

    /// Read data from the buffer synchronously (blocking)
    pub fn read_data_blocking<T: bytemuck::Pod>(&self, device: &GpuDevice) -> Result<Vec<T>> {
        pollster::block_on(self.read_data(device))
    }

    /// Get a binding resource for this buffer
    pub fn as_entire_binding(&self) -> BindingResource<'_> {
        self.buffer.as_entire_binding()
    }

    /// Get a binding resource for a slice of this buffer
    pub fn as_binding(&self, offset: u64, size: Option<u64>) -> BindingResource<'_> {
        BindingResource::Buffer(wgpu::BufferBinding {
            buffer: &self.buffer,
            offset,
            size: size.map(|s| NonZeroU64::new(s).unwrap()),
        })
    }
}

use std::num::NonZeroU64;
