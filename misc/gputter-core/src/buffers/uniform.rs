use crate::size::{BufferSize, DynamicSize, StaticSize};
use crate::{get_device, GpuDevice};

use super::{GpuBuffer, WritableGpuBuffer};

use crate::types::GpuType;

use std::marker::PhantomData;

/// Uniform Buffers can only be read from shaders, and written to by the host.
pub struct UniformBuffer<T: GpuType + ?Sized> {
    pub(crate) buffer: wgpu::Buffer,
    size: T::Size,
    pub(crate) phantom: PhantomData<T>,
}

impl<T: GpuType + ?Sized> GpuBuffer for UniformBuffer<T> {
    type HostHidden = Self;
    type PostSubmission<'a>
        = ()
    where
        Self: 'a;
    type Data = T;

    fn create_layout(binding: u32) -> wgpu::BindGroupLayoutEntry {
        wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }
    }
    fn get_buffer(&self) -> &wgpu::Buffer {
        &self.buffer
    }
    fn get_writable_size(&self) -> u64 {
        self.size.size()
    }
    fn pre_submission(&self, _encoder: &mut wgpu::CommandEncoder) {}
    fn post_submission(&self) -> Self::PostSubmission<'_> {}
}

impl<T: GpuType<Size = StaticSize<T>>> UniformBuffer<T> {
    pub fn new() -> Self {
        const {
            // If this assertion fails, the size of T
            // is too large to be used in a uniform buffer.
            assert!(size_of::<T>() < 65536);
        }
        let GpuDevice { device, .. } = get_device();
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: size_of::<T>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        Self {
            buffer,
            size: StaticSize::default(),
            phantom: PhantomData,
        }
    }
}

impl<T: GpuType + ?Sized> WritableGpuBuffer for UniformBuffer<T> {}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TooLargeForUniform;

impl std::fmt::Display for TooLargeForUniform {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Type is too large to be used in a uniform buffer (max 65536 bytes)"
        )
    }
}

impl std::error::Error for TooLargeForUniform {}

impl<T> UniformBuffer<[T]>
where
    [T]: GpuType<Size = DynamicSize<T>>,
{
    pub fn new_dyn(len: usize) -> Result<Self, TooLargeForUniform> {
        let size = len as u64 * size_of::<T>() as u64;
        if size < 65536 {
            let GpuDevice { device, .. } = get_device();
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: None,
                size,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            Ok(Self {
                buffer,
                size: DynamicSize::new(len),
                phantom: PhantomData,
            })
        } else {
            Err(TooLargeForUniform)
        }
    }
}
