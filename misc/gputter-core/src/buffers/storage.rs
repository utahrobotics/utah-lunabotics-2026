use super::{GpuBuffer, GpuWriteLock, WritableGpuBuffer};

use crate::size::{BufferSize, DynamicSize};

use crate::{get_device, GpuDevice};

use crate::size::StaticSize;

use std::marker::PhantomData;

use crate::types::GpuType;

pub trait HostStorageBufferMode {
    const HOST_CAN_READ: bool;
    fn get_usage() -> wgpu::BufferUsages;
}

#[derive(Clone, Copy, Debug)]
pub struct HostHidden;

impl HostStorageBufferMode for HostHidden {
    const HOST_CAN_READ: bool = false;
    fn get_usage() -> wgpu::BufferUsages {
        wgpu::BufferUsages::STORAGE
    }
}

#[derive(Clone, Copy, Debug)]
pub struct HostReadOnly;

impl HostStorageBufferMode for HostReadOnly {
    const HOST_CAN_READ: bool = true;
    fn get_usage() -> wgpu::BufferUsages {
        wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC
    }
}

#[derive(Clone, Copy, Debug)]
pub struct HostWriteOnly;

impl HostStorageBufferMode for HostWriteOnly {
    const HOST_CAN_READ: bool = false;
    fn get_usage() -> wgpu::BufferUsages {
        wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST
    }
}

#[derive(Clone, Copy, Debug)]
pub struct HostReadWrite;

impl HostStorageBufferMode for HostReadWrite {
    const HOST_CAN_READ: bool = true;
    fn get_usage() -> wgpu::BufferUsages {
        wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC | wgpu::BufferUsages::COPY_DST
    }
}

pub trait ShaderStorageBufferMode {
    const READONLY: bool;
}

#[derive(Clone, Copy, Debug)]
pub struct ShaderReadOnly;

impl ShaderStorageBufferMode for ShaderReadOnly {
    const READONLY: bool = true;
}

#[derive(Clone, Copy, Debug)]
pub struct ShaderReadWrite;

impl ShaderStorageBufferMode for ShaderReadWrite {
    const READONLY: bool = false;
}

pub struct StorageBuffer<T: GpuType + ?Sized, HM, SM> {
    pub(crate) buffer: wgpu::Buffer,
    pub(crate) size: T::Size,
    pub(crate) phantom: PhantomData<(fn() -> (HM, SM), T)>,
    read_buffer: Option<wgpu::Buffer>,
}

impl<T, HM, SM> StorageBuffer<T, HM, SM>
where
    T: GpuType<Size = StaticSize<T>>,
    HM: HostStorageBufferMode,
    SM: ShaderStorageBufferMode,
{
    pub fn new() -> Self {
        const {
            // If this assertion fails, the size of T
            // is too large to be used in a storage buffer.
            assert!(std::mem::size_of::<T>() < 134217728);
        }
        let GpuDevice { device, .. } = get_device();
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: size_of::<T>() as u64,
            usage: HM::get_usage(),
            mapped_at_creation: false,
        });

        // Only allocate a read buffer if the host can read and the shader can write
        let read_buffer = if const { HM::HOST_CAN_READ && !SM::READONLY } {
            Some(device.create_buffer(&wgpu::BufferDescriptor {
                label: None,
                size: size_of::<T>() as u64,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: true,
            }))
        } else {
            None
        };
        Self {
            read_buffer,
            buffer,
            size: StaticSize::default(),
            phantom: PhantomData,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TooLargeForStorage;

impl std::fmt::Display for TooLargeForStorage {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(
            f,
            "Type is too large to be used in a storage buffer (max 134217728 bytes)"
        )
    }
}

impl std::error::Error for TooLargeForStorage {}

impl<T, HM, SM> StorageBuffer<[T], HM, SM>
where
    [T]: GpuType<Size = DynamicSize<T>>,
    HM: HostStorageBufferMode,
    SM: ShaderStorageBufferMode,
{
    pub fn new_dyn(len: usize) -> Result<Self, TooLargeForStorage> {
        let size = len as u64 * size_of::<T>() as u64;
        if size < 134217728 {
            let GpuDevice { device, .. } = get_device();
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: None,
                size,
                usage: HM::get_usage(),
                mapped_at_creation: false,
            });
            // Only allocate a read buffer if the host can read and the shader can write
            let read_buffer = if const { HM::HOST_CAN_READ && !SM::READONLY } {
                Some(device.create_buffer(&wgpu::BufferDescriptor {
                    label: None,
                    size,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: true,
                }))
            } else {
                None
            };
            Ok(Self {
                read_buffer,
                buffer,
                size: DynamicSize::new(len),
                phantom: PhantomData,
            })
        } else {
            Err(TooLargeForStorage)
        }
    }
}

impl<T, SM> GpuBuffer for StorageBuffer<T, HostHidden, SM>
where
    T: GpuType + ?Sized,
    SM: ShaderStorageBufferMode,
{
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
                ty: wgpu::BufferBindingType::Storage {
                    read_only: SM::READONLY,
                },
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
        0
    }
    fn pre_submission(&self, _encoder: &mut wgpu::CommandEncoder) {
        debug_assert!(self.read_buffer.is_none());
    }
    fn post_submission(&self) -> Self::PostSubmission<'_> {
        debug_assert!(self.read_buffer.is_none());
    }
}

impl<T, SM> GpuBuffer for StorageBuffer<T, HostWriteOnly, SM>
where
    T: GpuType + ?Sized,
    SM: ShaderStorageBufferMode,
{
    type HostHidden = StorageBuffer<T, HostHidden, SM>;
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
                ty: wgpu::BufferBindingType::Storage {
                    read_only: SM::READONLY,
                },
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
    fn pre_submission(&self, _encoder: &mut wgpu::CommandEncoder) {
        debug_assert!(self.read_buffer.is_none());
    }
    fn post_submission(&self) -> Self::PostSubmission<'_> {
        debug_assert!(self.read_buffer.is_none());
    }
}

impl<T, SM> WritableGpuBuffer for StorageBuffer<T, HostWriteOnly, SM>
where
    T: GpuType + ?Sized,
    SM: ShaderStorageBufferMode,
{
}

macro_rules! read_impl {
    () => {
        fn pre_submission(&self, encoder: &mut wgpu::CommandEncoder) {
            let read_buffer = self.read_buffer.as_ref().unwrap();
            read_buffer.unmap();
            encoder.copy_buffer_to_buffer(&self.buffer, 0, read_buffer, 0, self.size.size());
        }
        fn post_submission(&self) -> Self::PostSubmission<'_> {
            let read_buffer = self.read_buffer.as_ref().unwrap();
            let slice = read_buffer.slice(..);
            slice.map_async(wgpu::MapMode::Read, |result| {
                result.unwrap();
            });
            slice
        }
    };
}

impl<T> GpuBuffer for StorageBuffer<T, HostReadOnly, ShaderReadWrite>
where
    T: GpuType + ?Sized,
{
    type HostHidden = StorageBuffer<T, HostHidden, ShaderReadWrite>;
    type PostSubmission<'a>
        = wgpu::BufferSlice<'a>
    where
        Self: 'a;
    type Data = T;

    fn create_layout(binding: u32) -> wgpu::BindGroupLayoutEntry {
        wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
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
        0
    }
    read_impl!();
}

impl<T> GpuBuffer for StorageBuffer<T, HostReadWrite, ShaderReadWrite>
where
    T: GpuType + ?Sized,
{
    type HostHidden = StorageBuffer<T, HostHidden, ShaderReadWrite>;
    type PostSubmission<'a>
        = wgpu::BufferSlice<'a>
    where
        Self: 'a;
    type Data = T;

    fn create_layout(binding: u32) -> wgpu::BindGroupLayoutEntry {
        wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only: false },
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
    read_impl!();
}

impl<T, SM> WritableGpuBuffer for StorageBuffer<T, HostReadWrite, SM>
where
    T: GpuType + ?Sized,
    SM: ShaderStorageBufferMode,
    Self: GpuBuffer,
{
}

impl<T, HM, SM> StorageBuffer<T, HM, SM>
where
    T: GpuType + ?Sized,
    HM: HostStorageBufferMode<HOST_CAN_READ = true>,
{
    pub fn read(&self, into: &mut T) {
        into.from_bytes(
            &self
                .read_buffer
                .as_ref()
                .unwrap()
                .slice(..)
                .get_mapped_range(),
        );
    }
    pub fn copy_into_unchecked(&self, other: &mut impl WritableGpuBuffer, lock: &mut GpuWriteLock) {
        lock.encoder.copy_buffer_to_buffer(
            &self.buffer,
            0,
            other.get_buffer(),
            0,
            self.size.size(),
        );
    }
}

impl<T, HM, SM> StorageBuffer<T, HM, SM>
where
    T: GpuType<Size = StaticSize<T>>,
{
    pub fn cast<U: GpuType<Size = StaticSize<U>>>(self) -> StorageBuffer<U, HM, SM> {
        const {
            if size_of::<T>() != size_of::<U>() {
                panic!("Attempted to cast between types of different sizes");
            }
        }
        StorageBuffer {
            buffer: self.buffer,
            size: StaticSize::new(),
            phantom: PhantomData,
            read_buffer: self.read_buffer,
        }
    }
}

impl<T, HM, SM> StorageBuffer<T, HM, SM>
where
    T: GpuType<Size = DynamicSize<T>>,
{
    pub fn cast_dyn<U: GpuType<Size = DynamicSize<U>>>(self) -> StorageBuffer<U, HM, SM> {
        const {
            if size_of::<T>() != size_of::<U>() {
                panic!("Attempted to cast between types of different sizes");
            }
            if align_of::<T>() != align_of::<U>() {
                panic!("Attempted to cast between types of different alignments");
            }
        }
        StorageBuffer {
            buffer: self.buffer,
            size: DynamicSize::new(self.size.0),
            phantom: PhantomData,
            read_buffer: self.read_buffer,
        }
    }
}
