use std::marker::PhantomData;

/// A trait for types that can accurately represent the size of a buffer.
pub trait BufferSize: Copy + Default + Send + 'static {
    fn size(&self) -> u64;
}

/// A buffer size that is statically known as `T` is statically sized.
pub struct StaticSize<T>(PhantomData<fn() -> T>);
impl<T> StaticSize<T> {
    pub const fn new() -> Self {
        Self(PhantomData)
    }
}

impl<T> Default for StaticSize<T> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<T> Copy for StaticSize<T> {}
impl<T> Clone for StaticSize<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T: 'static> BufferSize for StaticSize<T> {
    fn size(&self) -> u64 {
        size_of::<T>() as u64
    }
}

/// A buffer size that is a multiple of `T`, where `T` is statically sized.
///
/// Used for slices.
pub struct DynamicSize<T: ?Sized>(pub usize, PhantomData<fn() -> T>);

impl<T: 'static> BufferSize for DynamicSize<T> {
    fn size(&self) -> u64 {
        let stride = size_of::<T>() as u64;
        self.0 as u64 * stride
    }
}

impl<T: ?Sized> DynamicSize<T> {
    pub fn new(len: usize) -> Self {
        Self(len, PhantomData)
    }
}
impl<T: ?Sized> Default for DynamicSize<T> {
    fn default() -> Self {
        Self(0, PhantomData)
    }
}
impl<T: ?Sized> Copy for DynamicSize<T> {}
impl<T: ?Sized> Clone for DynamicSize<T> {
    fn clone(&self) -> Self {
        *self
    }
}
