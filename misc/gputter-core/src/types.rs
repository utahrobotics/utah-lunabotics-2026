//! Alignment on `wgpu` is annoying as it is vastly different from
//! how equivalent types are aligned in Rust. Refer to this website
//! for info https://webgpufundamentals.org/webgpu/lessons/webgpu-memory-layout.html

use std::ops::{Deref, DerefMut};

use bytemuck::{bytes_of, bytes_of_mut, cast_slice, cast_slice_mut, from_bytes};
use nalgebra::{Matrix2, Matrix3, Matrix4, Scalar, Vector2, Vector3, Vector4};

use crate::size::{BufferSize, DynamicSize, StaticSize};

pub trait GpuType {
    type Size: BufferSize;

    fn to_bytes(&self) -> &[u8];
    fn from_bytes(&mut self, bytes: &[u8]);
}

macro_rules! bytemuck_impl {
    ($type: ty) => {
        impl GpuType for $type {
            type Size = StaticSize<Self>;

            fn to_bytes(&self) -> &[u8] {
                bytes_of(self)
            }

            fn from_bytes(&mut self, bytes: &[u8]) {
                *self = *from_bytes(bytes);
            }
        }
    };
}

bytemuck_impl!(u32);
bytemuck_impl!(i32);
bytemuck_impl!(f32);

macro_rules! define_aligned {
    ($name: ident $align: literal $inner: ident $padding: literal $count: literal) => {
        #[derive(Clone, Copy, Debug)]
        #[repr(C)]
        #[repr(align($align))]
        pub struct $name<N> {
            pub vec: $inner<N>,
            _padding: [u8; $padding],
        }

        impl<N> From<$inner<N>> for $name<N> {
            fn from(vec: $inner<N>) -> Self {
                Self {
                    vec,
                    _padding: [0; $padding],
                }
            }
        }

        impl<N> From<$name<N>> for $inner<N> {
            fn from(v: $name<N>) -> Self {
                v.vec
            }
        }

        impl<N> Default for $name<N>
        where
            $inner<N>: Default,
        {
            fn default() -> Self {
                Self::from(<$inner<N>>::default())
            }
        }

        impl<N> Deref for $name<N> {
            type Target = $inner<N>;

            fn deref(&self) -> &Self::Target {
                &self.vec
            }
        }

        impl<N> DerefMut for $name<N> {
            fn deref_mut(&mut self) -> &mut Self::Target {
                &mut self.vec
            }
        }

        impl std::fmt::Display for $name<f32> {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "vec{}f(", $count)?;
                for i in 0..$count {
                    write!(f, "{}f,", self.vec.data.0[0][i])?;
                }
                write!(f, ")")?;
                Ok(())
            }
        }

        impl std::fmt::Display for $name<u32> {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "vec{}u(", $count)?;
                for i in 0..$count {
                    write!(f, "{}u,", self.vec.data.0[0][i])?;
                }
                write!(f, ")")?;
                Ok(())
            }
        }

        impl std::fmt::Display for $name<i32> {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                write!(f, "vec{}i(", $count)?;
                for i in 0..$count {
                    write!(f, "{}i,", self.vec.data.0[0][i])?;
                }
                write!(f, ")")?;
                Ok(())
            }
        }

        // These where clauses help to protect against unsafe behavior
        unsafe impl bytemuck::Zeroable for $name<u32> where $inner<u32>: bytemuck::Zeroable {}
        unsafe impl bytemuck::Pod for $name<u32> where $inner<u32>: bytemuck::Pod {}

        unsafe impl bytemuck::Zeroable for $name<f32> where $inner<f32>: bytemuck::Zeroable {}
        unsafe impl bytemuck::Pod for $name<f32> where $inner<f32>: bytemuck::Pod {}

        unsafe impl bytemuck::Zeroable for $name<i32> where $inner<i32>: bytemuck::Zeroable {}
        unsafe impl bytemuck::Pod for $name<i32> where $inner<i32>: bytemuck::Pod {}

        impl<N> GpuType for $name<N>
        where
            Self: bytemuck::Pod,
        {
            type Size = StaticSize<Self>;

            fn to_bytes(&self) -> &[u8] {
                bytes_of(self)
            }

            fn from_bytes(&mut self, bytes: &[u8]) {
                bytes_of_mut(self).copy_from_slice(bytes);
            }
        }
    };
}

define_aligned!(AlignedVec2 8 Vector2 8 2);
define_aligned!(AlignedVec3 16 Vector3 4 3);
define_aligned!(AlignedVec4 16 Vector4 0 4);

impl<const N: usize, T> GpuType for [T; N]
where
    Self: bytemuck::NoUninit + bytemuck::AnyBitPattern,
{
    type Size = StaticSize<Self>;

    fn to_bytes(&self) -> &[u8] {
        bytes_of(self)
    }

    fn from_bytes(&mut self, bytes: &[u8]) {
        bytes_of_mut(self).copy_from_slice(bytes);
    }
}

impl<T: 'static> GpuType for [T]
where
    T: bytemuck::NoUninit + bytemuck::AnyBitPattern,
{
    type Size = DynamicSize<T>;

    fn to_bytes(&self) -> &[u8] {
        cast_slice(self)
    }

    fn from_bytes(&mut self, bytes: &[u8]) {
        cast_slice_mut::<_, u8>(self).copy_from_slice(bytes);
    }
}

#[derive(Clone, Copy, Debug)]
pub struct AlignedMat<V, const C: usize> {
    pub columns: [V; C],
}

unsafe impl<V, const C: usize> bytemuck::Zeroable for AlignedMat<V, C> where
    [V; C]: bytemuck::Zeroable
{
}
unsafe impl<V, const C: usize> bytemuck::Pod for AlignedMat<V, C>
where
    [V; C]: bytemuck::Pod,
    V: Copy,
{
}
impl<V, const C: usize> GpuType for AlignedMat<V, C>
where
    Self: bytemuck::Pod,
{
    type Size = StaticSize<Self>;

    fn to_bytes(&self) -> &[u8] {
        bytes_of(self)
    }

    fn from_bytes(&mut self, bytes: &[u8]) {
        bytes_of_mut(self).copy_from_slice(bytes);
    }
}

macro_rules! define_mat {
    ($name: ident $vec: ident $columns: literal $matrix: ident) => {
        pub type $name<N> = AlignedMat<$vec<N>, $columns>;

        impl<N: Scalar> From<$matrix<N>> for $name<N> {
            fn from(m: $matrix<N>) -> Self {
                Self {
                    columns: std::array::from_fn(|i| $vec::from(m.column(i).into_owned())),
                }
            }
        }

        impl<N: Scalar + Copy> From<$name<N>> for $matrix<N> {
            fn from(m: $name<N>) -> Self {
                Self::from_iterator(m.columns.into_iter().flat_map(|v| v.vec.data.0[0]))
            }
        }
    };
}

define_mat!(AlignedMatrix2 AlignedVec2 2 Matrix2);
define_mat!(AlignedMatrix3 AlignedVec3 3 Matrix3);
define_mat!(AlignedMatrix4 AlignedVec4 4 Matrix4);

pub type AlignedMatrix2x2<N> = AlignedMatrix2<N>;
pub type AlignedMatrix3x3<N> = AlignedMatrix3<N>;
pub type AlignedMatrix4x4<N> = AlignedMatrix4<N>;
