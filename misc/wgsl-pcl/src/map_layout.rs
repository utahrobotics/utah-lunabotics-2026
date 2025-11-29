use crate::gpu_types::GpuType;
use crate::size::StaticSize;
use bytemuck::{bytes_of, bytes_of_mut};

/// All units are in meters
/// Struct is 16-byte aligned for WGPU compatibility
#[derive(Clone, Copy, Debug)]
#[repr(C)]
#[repr(align(16))]
pub struct MapLayout {
    pub max_x: f32,
    pub min_x: f32,
    pub max_y: f32,
    pub min_y: f32,
    pub cell_size: f32,
    _padding: [f32; 3],
}

impl MapLayout {
    pub fn new(max_x: f32, min_x: f32, max_y: f32, min_y: f32, cell_size: f32) -> Self {
        Self {
            max_x,
            min_x,
            max_y,
            min_y,
            cell_size,
            _padding: [0.0; 3],
        }
    }

    /// width in meters
    pub fn width_meters(&self) -> f32 {
        self.max_x - self.min_x
    }

    /// height in meters
    pub fn height_meters(&self) -> f32 {
        self.max_y - self.min_y
    }
}

// Safety: MapLayout contains only f32 values and padding
unsafe impl bytemuck::Zeroable for MapLayout {}
unsafe impl bytemuck::Pod for MapLayout {}

impl GpuType for MapLayout {
    type Size = StaticSize<Self>;

    fn to_bytes(&self) -> &[u8] {
        bytes_of(self)
    }

    fn from_bytes(&mut self, bytes: &[u8]) {
        bytes_of_mut(self).copy_from_slice(bytes);
    }
}
