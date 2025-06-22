use std::{
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    sync::Arc,
};

use crossbeam::queue::SegQueue;
use parking_lot::{Condvar, Mutex};

struct MonoQueue<T> {
    data: Mutex<Option<T>>,
    condvar: Condvar,
}

impl<T> Default for MonoQueue<T> {
    fn default() -> Self {
        Self {
            data: Mutex::new(None),
            condvar: Condvar::new(),
        }
    }
}

impl<T> MonoQueue<T> {
    fn try_clear(&self) {
        if let Some(mut guard) = self.data.try_lock() {
            *guard = None;
        }
    }

    fn set(&self, data: T) {
        {
            let mut lock = self.data.lock();
            *lock = Some(data);
        }
        self.condvar.notify_one();
    }

    fn get(&self) -> T {
        let mut lock = self.data.lock();
        if lock.is_none() {
            self.condvar.wait_while(&mut lock, |inner| inner.is_none());
        }
        unsafe { lock.take().unwrap_unchecked() }
    }

    fn try_get(&self) -> Option<T> {
        let mut lock = self.data.lock();
        lock.take()
    }
}

struct DataInner<T> {
    data: MaybeUninit<T>,
    released_mut: Mutex<()>,
    released_condvar: Condvar,
}

struct DataHandleInner<T> {
    new_callbacks: SegQueue<Box<dyn FnMut(&T) + Send>>,
    new_lendees: SegQueue<Arc<MonoQueue<Arc<DataInner<T>>>>>,
}

/// A handle to a shared data object.
///
/// You can add callbacks to be called when the data is updated,
/// and create lendees to receive the data.
#[repr(transparent)]
#[derive(Clone)]
pub struct DataHandle<T> {
    inner: Arc<DataHandleInner<T>>,
}

impl<T> DataHandle<T> {
    /// Adds a callback to be called when [`OwnedData::share`] is called.
    pub fn add_callback(&self, callback: impl FnMut(&T) + Send + 'static) {
        self.inner.new_callbacks.push(Box::new(callback));
    }

    /// Creates a lendee to receive the data when [`OwnedData::share`] is called.
    pub fn create_lendee(&self) -> SharedDataReceiver<T> {
        let queue: Arc<MonoQueue<Arc<DataInner<T>>>> = Default::default();
        self.inner.new_lendees.push(queue.clone());
        SharedDataReceiver { queue }
    }
}

/// An uninitialized heap allocation for `T`.
pub struct UninitOwnedData<T> {
    inner: Arc<DataInner<T>>,
    callbacks: Vec<Box<dyn FnMut(&T) + Send>>,
    data_handle: Arc<DataHandleInner<T>>,
    lendees: Vec<Arc<MonoQueue<Arc<DataInner<T>>>>>,
}

impl<T> UninitOwnedData<T> {
    /// Adds a callback to be called when [`share`] is called.
    pub fn add_callback(&mut self, callback: impl FnMut(&T) + Send + 'static) {
        self.callbacks.push(Box::new(callback));
    }

    /// Creates a lendee to receive the data when [`share`] is called.
    pub fn create_lendee(&mut self) -> SharedDataReceiver<T> {
        let queue: Arc<MonoQueue<Arc<DataInner<T>>>> = Default::default();
        self.lendees.push(queue.clone());
        SharedDataReceiver { queue }
    }

    /// Returns a reference to the handle.
    pub fn get_data_handle(&self) -> &DataHandle<T> {
        unsafe { std::mem::transmute(&self.data_handle) }
    }

    /// Initializes the data.
    pub fn init(mut self, data: T) -> OwnedData<T> {
        unsafe {
            Arc::get_mut_unchecked(&mut self.inner).data.write(data);
        }
        OwnedData {
            inner: self.inner,
            callbacks: self.callbacks,
            data_handle: self.data_handle,
            lendees: self.lendees,
        }
    }
}

impl<T> Default for UninitOwnedData<T> {
    fn default() -> Self {
        Self {
            inner: Arc::new(DataInner {
                data: MaybeUninit::uninit(),
                released_condvar: Condvar::new(),
                released_mut: Mutex::new(()),
            }),
            callbacks: Vec::new(),
            data_handle: Arc::new(DataHandleInner {
                new_callbacks: SegQueue::new(),
                new_lendees: SegQueue::new(),
            }),
            lendees: Vec::new(),
        }
    }
}

/// A smart pointer to `T` that can invoke callbacks
/// and temporarily lend the data to other threads.
pub struct OwnedData<T> {
    inner: Arc<DataInner<T>>,
    callbacks: Vec<Box<dyn FnMut(&T) + Send>>,
    data_handle: Arc<DataHandleInner<T>>,
    lendees: Vec<Arc<MonoQueue<Arc<DataInner<T>>>>>,
}

impl<T> From<T> for OwnedData<T> {
    fn from(value: T) -> Self {
        Self {
            inner: Arc::new(DataInner {
                data: MaybeUninit::new(value),
                released_condvar: Condvar::new(),
                released_mut: Mutex::new(()),
            }),
            callbacks: Vec::new(),
            data_handle: Arc::new(DataHandleInner {
                new_callbacks: SegQueue::new(),
                new_lendees: SegQueue::new(),
            }),
            lendees: Vec::new(),
        }
    }
}

impl<T> Deref for OwnedData<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { self.inner.data.assume_init_ref() }
    }
}

impl<T> DerefMut for OwnedData<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        unsafe {
            Arc::get_mut_unchecked(&mut self.inner)
                .data
                .assume_init_mut()
        }
    }
}

impl<T> Drop for OwnedData<T> {
    fn drop(&mut self) {
        unsafe {
            Arc::get_mut_unchecked(&mut self.inner)
                .data
                .assume_init_drop()
        };
    }
}

impl<T> OwnedData<T> {
    /// Unwraps the data.
    pub fn unwrap(self) -> T {
        unsafe { self.inner.data.assume_init_read() }
    }

    /// Unitinializes self and returns the data.
    pub fn uninit(self) -> (T, UninitOwnedData<T>) {
        unsafe {
            let data = self.inner.data.assume_init_read();
            let tmp = MaybeUninit::new(self);
            let tmp = tmp.as_ptr();
            let inner = &raw const (*tmp).inner;
            let callbacks = &raw const (*tmp).callbacks;
            let data_handle = &raw const (*tmp).data_handle;
            let lendees = &raw const (*tmp).lendees;
            (
                data,
                UninitOwnedData {
                    inner: inner.read(),
                    callbacks: callbacks.read(),
                    data_handle: data_handle.read(),
                    lendees: lendees.read(),
                },
            )
        }
    }

    /// Adds a callback to be called when [`share`] is called.
    pub fn add_callback(&mut self, callback: impl FnMut(&T) + Send + 'static) {
        self.callbacks.push(Box::new(callback));
    }

    /// Creates a lendee to receive the data when [`share`] is called.
    pub fn create_lendee(&mut self) -> SharedDataReceiver<T> {
        let queue: Arc<MonoQueue<Arc<DataInner<T>>>> = Default::default();
        self.lendees.push(queue.clone());
        SharedDataReceiver { queue }
    }

    /// Returns a reference to the handle.
    pub fn get_data_handle(&self) -> &DataHandle<T> {
        unsafe { std::mem::transmute(&self.data_handle) }
    }

    /// Invokes callbacks and lends the data to other threads.
    ///
    /// Always assumes that ownership was shared.
    pub fn pessimistic_share(mut self) -> LoanedData<T> {
        self.callbacks.reserve(self.data_handle.new_callbacks.len());
        while let Some(callback) = self.data_handle.new_callbacks.pop() {
            self.callbacks.push(callback);
        }

        for callback in &mut self.callbacks {
            callback(unsafe { self.inner.data.assume_init_ref() });
        }

        self.lendees.reserve(self.data_handle.new_lendees.len());
        while let Some(lendee) = self.data_handle.new_lendees.pop() {
            self.lendees.push(lendee);
        }

        for lendee in &self.lendees {
            lendee.set(self.inner.clone());
        }

        unsafe {
            let tmp = MaybeUninit::new(self);
            let tmp = tmp.as_ptr();
            let inner = &raw const (*tmp).inner;
            let callbacks = &raw const (*tmp).callbacks;
            let data_handle = &raw const (*tmp).data_handle;
            let lendees = &raw const (*tmp).lendees;

            LoanedData {
                inner: inner.read(),
                data_handle: data_handle.read(),
                callbacks: callbacks.read(),
                lendees: lendees.read(),
            }
        }
    }

    /// Invokes callbacks and lends the data to other threads.
    ///
    /// If no threads were registered to receive the data, ownership is returned.
    /// Otherwise, a `LoanedData` object is returned which only allows immutable access.
    pub fn share(mut self) -> Result<Self, LoanedData<T>> {
        self.callbacks.reserve(self.data_handle.new_callbacks.len());
        while let Some(callback) = self.data_handle.new_callbacks.pop() {
            self.callbacks.push(callback);
        }

        for callback in &mut self.callbacks {
            callback(unsafe { self.inner.data.assume_init_ref() });
        }

        self.lendees.reserve(self.data_handle.new_lendees.len());
        while let Some(lendee) = self.data_handle.new_lendees.pop() {
            self.lendees.push(lendee);
        }

        if self.lendees.is_empty() {
            Ok(self)
        } else {
            for lendee in &self.lendees {
                lendee.set(self.inner.clone());
            }

            unsafe {
                let tmp = MaybeUninit::new(self);
                let tmp = tmp.as_ptr();
                let inner = &raw const (*tmp).inner;
                let callbacks = &raw const (*tmp).callbacks;
                let data_handle = &raw const (*tmp).data_handle;
                let lendees = &raw const (*tmp).lendees;

                Err(LoanedData {
                    inner: inner.read(),
                    data_handle: data_handle.read(),
                    callbacks: callbacks.read(),
                    lendees: lendees.read(),
                })
            }
        }
    }
}

/// A smart pointer to `T` that only allows immutable access.
pub struct LoanedData<T> {
    inner: Arc<DataInner<T>>,
    data_handle: Arc<DataHandleInner<T>>,
    callbacks: Vec<Box<dyn FnMut(&T) + Send>>,
    lendees: Vec<Arc<MonoQueue<Arc<DataInner<T>>>>>,
}

impl<T> LoanedData<T> {
    /// Adds a callback to be called when [`share`] is called.
    pub fn add_callback(&mut self, callback: impl FnMut(&T) + Send + 'static) {
        self.callbacks.push(Box::new(callback));
    }

    /// Creates a lendee to receive the data when [`share`] is called.
    pub fn create_lendee(&mut self) -> SharedDataReceiver<T> {
        let queue: Arc<MonoQueue<Arc<DataInner<T>>>> = Default::default();
        self.lendees.push(queue.clone());
        SharedDataReceiver { queue }
    }

    /// Returns a reference to the handle.
    pub fn get_data_handle(&self) -> &DataHandle<T> {
        unsafe { std::mem::transmute(&self.data_handle) }
    }

    /// Waits for other threads to drop their ownership of the data.
    pub fn recall(self) -> OwnedData<T> {
        for lendee in &self.lendees {
            lendee.try_clear();
        }
        {
            let mut lock = self.inner.released_mut.lock();
            if Arc::strong_count(&self.inner) > 1 {
                self.inner.released_condvar.wait(&mut lock);
                debug_assert_eq!(Arc::strong_count(&self.inner), 1);
            }
        }

        unsafe {
            let tmp = MaybeUninit::new(self);
            let tmp = tmp.as_ptr();
            let inner = &raw const (*tmp).inner;
            let callbacks = &raw const (*tmp).callbacks;
            let data_handle = &raw const (*tmp).data_handle;
            let lendees = &raw const (*tmp).lendees;

            OwnedData {
                inner: inner.read(),
                data_handle: data_handle.read(),
                callbacks: callbacks.read(),
                lendees: lendees.read(),
            }
        }
    }

    /// Waits for other threads to drop their ownership of the data.
    pub fn try_recall(self) -> Result<OwnedData<T>, Self> {
        for lendee in &self.lendees {
            lendee.try_clear();
        }
        {
            let lock = self.inner.released_mut.lock();
            if Arc::strong_count(&self.inner) > 1 {
                drop(lock);
                return Err(self);
            }
        }

        unsafe {
            let tmp = MaybeUninit::new(self);
            let tmp = tmp.as_ptr();
            let inner = &raw const (*tmp).inner;
            let callbacks = &raw const (*tmp).callbacks;
            let data_handle = &raw const (*tmp).data_handle;
            let lendees = &raw const (*tmp).lendees;

            Ok(OwnedData {
                inner: inner.read(),
                data_handle: data_handle.read(),
                callbacks: callbacks.read(),
                lendees: lendees.read(),
            })
        }
    }

    /// Checks if other threads have dropped their ownership of the data, replacing
    /// data in-place if possible. Otherwise, ownership is replaced with `new_data`.
    pub fn replace(self, new_data: T) -> OwnedData<T> {
        for lendee in &self.lendees {
            lendee.try_clear();
        }
        if Arc::strong_count(&self.inner) == 1 {
            let mut owned = unsafe {
                let tmp = MaybeUninit::new(self);
                let tmp = tmp.as_ptr();
                let inner = &raw const (*tmp).inner;
                let callbacks = &raw const (*tmp).callbacks;
                let data_handle = &raw const (*tmp).data_handle;
                let lendees = &raw const (*tmp).lendees;

                OwnedData {
                    inner: inner.read(),
                    data_handle: data_handle.read(),
                    callbacks: callbacks.read(),
                    lendees: lendees.read(),
                }
            };
            *owned = new_data;
            return owned;
        }

        let inner = Arc::new(DataInner {
            data: MaybeUninit::new(new_data),
            released_condvar: Condvar::new(),
            released_mut: Mutex::new(()),
        });

        unsafe {
            let tmp = MaybeUninit::new(self);
            let tmp = tmp.as_ptr();
            let callbacks = &raw const (*tmp).callbacks;
            let data_handle = &raw const (*tmp).data_handle;
            let lendees = &raw const (*tmp).lendees;

            OwnedData {
                inner,
                data_handle: data_handle.read(),
                callbacks: callbacks.read(),
                lendees: lendees.read(),
            }
        }
    }

    /// Checks if other threads have dropped their ownership of the data, replacing
    /// data if still shared. Otherwise, ownership of the original data is returned.
    pub fn recall_or_replace_with(self, f: impl FnOnce() -> T) -> OwnedData<T> {
        for lendee in &self.lendees {
            lendee.try_clear();
        }
        if Arc::strong_count(&self.inner) == 1 {
            let owned = unsafe {
                let tmp = MaybeUninit::new(self);
                let tmp = tmp.as_ptr();
                let inner = &raw const (*tmp).inner;
                let callbacks = &raw const (*tmp).callbacks;
                let data_handle = &raw const (*tmp).data_handle;
                let lendees = &raw const (*tmp).lendees;

                OwnedData {
                    inner: inner.read(),
                    data_handle: data_handle.read(),
                    callbacks: callbacks.read(),
                    lendees: lendees.read(),
                }
            };
            return owned;
        }

        let inner = Arc::new(DataInner {
            data: MaybeUninit::new(f()),
            released_condvar: Condvar::new(),
            released_mut: Mutex::new(()),
        });

        unsafe {
            let tmp = MaybeUninit::new(self);
            let tmp = tmp.as_ptr();
            let callbacks = &raw const (*tmp).callbacks;
            let data_handle = &raw const (*tmp).data_handle;
            let lendees = &raw const (*tmp).lendees;

            OwnedData {
                inner,
                data_handle: data_handle.read(),
                callbacks: callbacks.read(),
                lendees: lendees.read(),
            }
        }
    }

    /// Creates a new uninit object that maintains the callbacks and lendees.
    ///
    /// This is different from [`OwnedData::uninit`], which reuses the heap allocation. This
    /// makes a new heap allocation.
    pub fn deinit(self) -> UninitOwnedData<T> {
        let inner = Arc::new(DataInner {
            data: MaybeUninit::uninit(),
            released_condvar: Condvar::new(),
            released_mut: Mutex::new(()),
        });
        unsafe {
            let tmp = MaybeUninit::new(self);
            let tmp = tmp.as_ptr();
            let callbacks = &raw const (*tmp).callbacks;
            let data_handle = &raw const (*tmp).data_handle;
            let lendees = &raw const (*tmp).lendees;

            UninitOwnedData {
                inner,
                data_handle: data_handle.read(),
                callbacks: callbacks.read(),
                lendees: lendees.read(),
            }
        }
    }
}

impl<T> Deref for LoanedData<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { self.inner.data.assume_init_ref() }
    }
}

impl<T> Drop for LoanedData<T> {
    fn drop(&mut self) {
        let inner = std::mem::replace(
            &mut self.inner,
            Arc::new(DataInner {
                data: MaybeUninit::uninit(),
                released_condvar: Condvar::new(),
                released_mut: Mutex::new(()),
            }),
        );
        if let Ok(mut tmp) = Arc::try_unwrap(inner) {
            unsafe { tmp.data.assume_init_drop() };
        }
    }
}

/// Temporary shared ownership of `T`.
///
/// In most cases, a thread is waiting on all `SharedData` objects to be dropped,
/// so dropping this object as early as possible is recommended.
pub struct SharedData<T> {
    inner: Arc<DataInner<T>>,
}

impl<T> Deref for SharedData<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        unsafe { self.inner.data.assume_init_ref() }
    }
}

impl<T> Drop for SharedData<T> {
    fn drop(&mut self) {
        let inner = std::mem::replace(
            &mut self.inner,
            Arc::new(DataInner {
                data: MaybeUninit::uninit(),
                released_condvar: Condvar::new(),
                released_mut: Mutex::new(()),
            }),
        );
        if let Ok(mut tmp) = Arc::try_unwrap(inner) {
            unsafe { tmp.data.assume_init_drop() };
        } else if Arc::strong_count(&self.inner) == 2 {
            let _guard = self.inner.released_mut.lock();
            self.inner.released_condvar.notify_one();
        }
    }
}

/// A receiver for shared data.
pub struct SharedDataReceiver<T> {
    queue: Arc<MonoQueue<Arc<DataInner<T>>>>,
}

impl<T> SharedDataReceiver<T> {
    /// Waits until the provider shares the data.
    ///
    /// If the provider has already shared the data, the data is returned immediately.
    /// If the provider provides, then recalls, before this method is called, this method will
    /// wait until the provider shares the data again.
    pub fn get(&self) -> SharedData<T> {
        SharedData {
            inner: self.queue.get(),
        }
    }

    /// Tries to get the data without waiting.
    pub fn try_get(&self) -> Option<SharedData<T>> {
        Some(SharedData {
            inner: self.queue.try_get()?,
        })
    }
}

/// A convenience type for handling both owned and loaned data under the same type.
pub enum MaybeOwned<T> {
    Owned(OwnedData<T>),
    Loaned(LoanedData<T>),
}

impl<T> MaybeOwned<T> {
    /// Tries to unwrap the data if owned.
    pub fn try_unwrap(self) -> Result<T, Self> {
        match self {
            Self::Owned(owned) => Ok(owned.unwrap()),
            Self::Loaned(loaned) => Err(Self::Loaned(loaned)),
        }
    }

    /// Returns a mutable reference to the data if owned.
    pub fn get_mut(&mut self) -> Option<&mut T> {
        match self {
            Self::Owned(owned) => Some(owned),
            _ => None,
        }
    }

    /// Adds a callback to be called when [`share`] is called.
    pub fn add_callback(&mut self, callback: impl FnMut(&T) + Send + 'static) {
        match self {
            Self::Owned(owned) => owned.add_callback(callback),
            Self::Loaned(loaned) => loaned.add_callback(callback),
        }
    }

    /// Creates a lendee to receive the data when [`share`] is called.
    pub fn create_lendee(&mut self) -> SharedDataReceiver<T> {
        match self {
            Self::Owned(owned) => owned.create_lendee(),
            Self::Loaned(loaned) => loaned.create_lendee(),
        }
    }

    /// Returns a reference to the handle.
    pub fn get_data_handle(&self) -> &DataHandle<T> {
        match self {
            Self::Owned(owned) => owned.get_data_handle(),
            Self::Loaned(loaned) => loaned.get_data_handle(),
        }
    }

    /// Invokes callbacks and lends the data to other threads.
    ///
    /// If no threads were registered to receive the data, ownership is returned.
    /// Otherwise, a `LoanedData` object is returned which only allows immutable access.
    pub fn share(&mut self) {
        match self {
            Self::Owned(owned) => unsafe {
                let owned_owned = std::ptr::read(owned);
                match owned_owned.share() {
                    Ok(x) => {
                        std::ptr::write(owned, x);
                    }
                    Err(x) => {
                        std::ptr::write(self, Self::Loaned(x));
                    }
                }
            },
            Self::Loaned(_) => {}
        }
    }

    /// Checks if other threads have dropped their ownership of the data, replacing
    /// data in-place if possible. Otherwise, ownership is replaced with `new_data`.
    pub fn replace(&mut self, new_data: T) {
        match self {
            Self::Owned(owned) => {
                *owned.deref_mut() = new_data;
            }
            Self::Loaned(loaned) => unsafe {
                let owned_loaded = std::ptr::read(loaned);
                let owned = owned_loaded.replace(new_data);
                std::ptr::write(self, Self::Owned(owned));
            },
        }
    }

    /// Waits for other threads to drop their ownership of the data.
    pub fn recall(&mut self) {
        match self {
            Self::Owned(_) => {}
            Self::Loaned(loaned) => unsafe {
                let owned_loaded = std::ptr::read(loaned);
                let owned = owned_loaded.recall();
                std::ptr::write(self, Self::Owned(owned));
            },
        }
    }

    pub fn try_recall(&mut self) -> bool {
        match self {
            Self::Owned(_) => true,
            Self::Loaned(loaned) => unsafe {
                let owned_loaded = std::ptr::read(loaned);
                match owned_loaded.try_recall() {
                    Ok(owned) => {
                        std::ptr::write(self, Self::Owned(owned));
                        true
                    }
                    Err(loaned) => {
                        std::ptr::write(self, Self::Loaned(loaned));
                        false
                    }
                }
            },
        }
    }
}

impl<T> Deref for MaybeOwned<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        match self {
            Self::Owned(owned) => owned,
            Self::Loaned(loaned) => loaned,
        }
    }
}

impl<T> From<T> for MaybeOwned<T> {
    fn from(value: T) -> Self {
        Self::Owned(value.into())
    }
}

impl<T> From<OwnedData<T>> for MaybeOwned<T> {
    fn from(value: OwnedData<T>) -> Self {
        Self::Owned(value)
    }
}

impl<T> From<LoanedData<T>> for MaybeOwned<T> {
    fn from(value: LoanedData<T>) -> Self {
        Self::Loaned(value)
    }
}

#[cfg(test)]
mod tests {
    use super::MaybeOwned;
    use std::sync::atomic::AtomicUsize;
    use std::sync::Arc;

    #[test]
    fn test01() {
        let mut data: MaybeOwned<i32> = 5.into();
        let counter = Arc::new(AtomicUsize::new(0));
        let counter2 = counter.clone();
        data.add_callback(move |&x| {
            assert_eq!(x, 5);
            counter2.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        });
        data.share();
        data.recall();
        data.share();
        data.recall();
        assert_eq!(counter.load(std::sync::atomic::Ordering::Relaxed), 2);
    }

    #[test]
    fn test02() {
        let mut data: MaybeOwned<i32> = 5.into();
        let user1 = data.create_lendee();
        let user2 = data.create_lendee();

        data.share();
        assert_eq!(user1.get().abs(), 5);
        assert_eq!(user2.get().abs(), 5);
    }
}
