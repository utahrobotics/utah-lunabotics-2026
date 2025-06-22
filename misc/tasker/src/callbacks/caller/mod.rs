use std::{
    cell::Cell,
    sync::{Arc, Weak},
};

use crossbeam::queue::SegQueue;
use parking_lot::{Mutex, RwLock};

pub mod ext;

thread_local! {
    static RETAIN_CALLBACK: Cell<bool> = const { Cell::new(false) };
}

pub fn try_drop_this_callback() {
    RETAIN_CALLBACK.set(false);
}

pub fn retain_this_callback() {
    RETAIN_CALLBACK.set(true);
}

pub mod prelude {
    pub use super::{
        retain_this_callback, try_drop_this_callback, Callback, CallbacksRef, CallbacksStorage,
        RawCallbackStorage,
    };
}

#[macro_export]
macro_rules! define_callbacks {
    ($vis: vis $name: ident => Fn($($param: ident : $arg: ty),*)$($extra: tt)*) => {
        #[allow(dead_code)]
        use $crate::callbacks::caller::prelude::*;

        #[derive(Default)]
        $vis struct $name {
            storage: RawCallbackStorage<dyn Fn($($arg,)*)$($extra)*, dyn FnMut($($arg,)*)$($extra)*>,
        }

        impl $name {
            /// Calls all callbacks.
            ///
            /// This *does not* drop any callbacks that return `true` from `was_callback_dropped` and is thus
            /// faster than `call`.
            #[allow(dead_code)]
            $vis fn call_immut(&self, $($param : $arg),*) {
                self.storage.for_each_immut(|callback| {
                    match callback {
                        Callback::Immut(func) => func($($param),*),
                        Callback::Mut(func) => (func.lock())($($param),*),
                    }
                });
            }

            /// Calls all callbacks.
            ///
            /// This drops any callbacks that return `true` from `was_callback_dropped`, unlike `call_immut`.
            #[allow(dead_code)]
            $vis fn call(&mut self, $($param : $arg),*) {
                self.storage.for_each(|callback| {
                    match callback {
                        Callback::Immut(func) => func($($param),*),
                        Callback::Mut(func) => (func.get_mut())($($param),*),
                    }
                });
            }
        }

        impl CallbacksStorage for $name {
            type Immut = dyn Fn($($arg,)*)$($extra)*;
            type Mut = dyn FnMut($($arg,)*)$($extra)*;

            fn get_storage(&self) -> &RawCallbackStorage<Self::Immut, Self::Mut> {
                &self.storage
            }

            fn get_storage_mut(&mut self) -> &mut RawCallbackStorage<Self::Immut, Self::Mut> {
                &mut self.storage
            }
        }
    };
    ($vis: vis $name: ident => CloneFn($($param: ident : $arg: ty),*)$($extra: tt)*) => {
        #[allow(dead_code)]
        use $crate::callbacks::caller::prelude::*;

        #[derive(Default)]
        $vis struct $name {
            storage: RawCallbackStorage<dyn Fn($($arg,)*)$($extra)*, dyn FnMut($($arg,)*)$($extra)*>,
        }

        impl $name {
            /// Calls all callbacks.
            ///
            /// This *does not* drop any callbacks that return `true` from `was_callback_dropped` and is thus
            /// faster than `call`.
            #[allow(dead_code)]
            $vis fn call_immut(&self, $($param : $arg),*) {
                self.storage.for_each_immut(|callback| {
                    match callback {
                        Callback::Immut(func) => func($(Clone::clone(&$param)),*),
                        Callback::Mut(func) => (func.lock())($(Clone::clone(&$param)),*),
                    }
                });
            }

            /// Calls all callbacks.
            ///
            /// This drops any callbacks that return `true` from `was_callback_dropped`, unlike `call_immut`.
            #[allow(dead_code)]
            $vis fn call(&mut self, $($param : $arg),*) {
                self.storage.for_each(|callback| {
                    match callback {
                        Callback::Immut(func) => func($(Clone::clone(&$param)),*),
                        Callback::Mut(func) => (func.get_mut())($(Clone::clone(&$param)),*),
                    }
                });
            }
        }

        impl CallbacksStorage for $name {
            type Immut = dyn Fn($($arg,)*)$($extra)*;
            type Mut = dyn FnMut($($arg,)*)$($extra)*;

            fn get_storage(&self) -> &RawCallbackStorage<Self::Immut, Self::Mut> {
                &self.storage
            }

            fn get_storage_mut(&mut self) -> &mut RawCallbackStorage<Self::Immut, Self::Mut> {
                &mut self.storage
            }
        }
    };
}

pub enum Callback<A: ?Sized, B: ?Sized> {
    Immut(Box<A>),
    Mut(Mutex<Box<B>>),
}

pub struct RawCallbackStorage<A: ?Sized, B: ?Sized> {
    incoming: Arc<SegQueue<Callback<A, B>>>,
    storage: RwLock<Vec<Callback<A, B>>>,
}

impl<A: ?Sized, B: ?Sized> Default for RawCallbackStorage<A, B> {
    fn default() -> Self {
        Self {
            incoming: Default::default(),
            storage: Default::default(),
        }
    }
}

impl<A: ?Sized, B: ?Sized> RawCallbackStorage<A, B> {
    pub fn for_each(&mut self, mut f: impl FnMut(&mut Callback<A, B>)) {
        self.storage.get_mut().retain_mut(|callback| {
            RETAIN_CALLBACK.with(|cell| cell.set(true));
            f(callback);
            RETAIN_CALLBACK.get()
        });
        while let Some(mut callback) = self.incoming.pop() {
            RETAIN_CALLBACK.with(|cell| cell.set(true));
            f(&mut callback);
            if RETAIN_CALLBACK.get() {
                self.storage.get_mut().push(callback);
            }
        }
    }

    pub fn for_each_immut(&self, mut f: impl FnMut(&Callback<A, B>)) {
        if let Some(mut storage) = self.storage.try_write() {
            storage.retain(|callback| {
                RETAIN_CALLBACK.with(|cell| cell.set(true));
                f(callback);
                RETAIN_CALLBACK.get()
            });
            while let Some(callback) = self.incoming.pop() {
                RETAIN_CALLBACK.with(|cell| cell.set(true));
                f(&callback);
                if RETAIN_CALLBACK.get() {
                    storage.push(callback);
                }
            }
        } else {
            let storage = self.storage.read();
            storage.iter().for_each(|callback| f(callback));
            for _ in 0..self.incoming.len() {
                let callback = self.incoming.pop().unwrap();
                RETAIN_CALLBACK.with(|cell| cell.set(true));
                f(&callback);
                if RETAIN_CALLBACK.get() {
                    self.incoming.push(callback);
                }
            }
        }
    }
}

pub trait CallbacksStorage {
    type Immut: ?Sized;
    type Mut: ?Sized;

    fn get_storage(&self) -> &RawCallbackStorage<Self::Immut, Self::Mut>;
    fn get_storage_mut(&mut self) -> &mut RawCallbackStorage<Self::Immut, Self::Mut>;

    fn add_dyn_fn(&mut self, callback: Box<Self::Immut>) {
        self.get_storage_mut()
            .storage
            .get_mut()
            .push(Callback::Immut(callback));
    }

    fn add_dyn_fn_mut(&mut self, callback: Box<Self::Mut>) {
        self.get_storage_mut()
            .storage
            .get_mut()
            .push(Callback::Mut(Mutex::new(callback)));
    }

    fn add_dyn_fn_immut(&self, callback: Box<Self::Immut>) {
        self.get_storage()
            .storage
            .write()
            .push(Callback::Immut(callback));
    }

    fn add_dyn_fn_mut_immut(&self, callback: Box<Self::Mut>) {
        self.get_storage()
            .storage
            .write()
            .push(Callback::Mut(Mutex::new(callback)));
    }

    fn is_empty(&self) -> bool {
        self.get_storage().storage.read().is_empty() && self.get_storage().incoming.is_empty()
    }

    fn len(&self) -> usize {
        self.get_storage().storage.read().len() + self.get_storage().incoming.len()
    }

    fn is_empty_mut(&mut self) -> bool {
        self.get_storage_mut().storage.get_mut().is_empty()
            && self.get_storage().incoming.is_empty()
    }

    fn size_mut(&mut self) -> usize {
        self.get_storage_mut().storage.get_mut().len() + self.get_storage().incoming.len()
    }

    /// Gets a reference that can be used to add callbacks.
    #[inline]
    fn get_ref(&self) -> CallbacksRef<Self::Immut, Self::Mut> {
        CallbacksRef {
            incoming: Arc::downgrade(&self.get_storage().incoming),
        }
    }
}

pub struct CallbacksRef<A: ?Sized, B: ?Sized> {
    incoming: Weak<SegQueue<Callback<A, B>>>,
}

impl<A: ?Sized, B: ?Sized> CallbacksRef<A, B> {
    pub fn add_dyn_fn(&self, callback: Box<A>) -> Option<Box<A>> {
        if let Some(incoming) = self.incoming.upgrade() {
            incoming.push(Callback::Immut(callback));
            None
        } else {
            Some(callback)
        }
    }

    pub fn add_dyn_fn_mut(&self, callback: Box<B>) -> Option<Box<B>> {
        if let Some(incoming) = self.incoming.upgrade() {
            incoming.push(Callback::Mut(Mutex::new(callback)));
            None
        } else {
            Some(callback)
        }
    }
}
impl<A: ?Sized, B: ?Sized> Clone for CallbacksRef<A, B> {
    fn clone(&self) -> Self {
        Self {
            incoming: self.incoming.clone(),
        }
    }
}

macro_rules! dyn_ref_impl {
    ($($extra: tt)*) => {
        pub fn add_fn<F: Fn(Args)$($extra)* + 'static>(&self, callback: F) -> Option<F> {
            if let Some(incoming) = self.incoming.upgrade() {
                incoming.push(Callback::Immut(Box::new(callback)));
                None
            } else {
                Some(callback)
            }
        }

        pub fn add_fn_mut<F: FnMut(Args)$($extra)* + 'static>(&self, callback: F) -> Option<F> {
            if let Some(incoming) = self.incoming.upgrade() {
                incoming.push(Callback::Mut(Mutex::new(Box::new(callback))));
                None
            } else {
                Some(callback)
            }
        }
    }
}

impl<Args> CallbacksRef<dyn Fn(Args), dyn FnMut(Args)> {
    dyn_ref_impl!();
}

impl<Args> CallbacksRef<dyn Fn(Args) + Send, dyn FnMut(Args) + Send> {
    dyn_ref_impl!( + Send);
}

impl<Args> CallbacksRef<dyn Fn(Args) + Sync, dyn FnMut(Args) + Sync> {
    dyn_ref_impl!( + Sync);
}

impl<Args> CallbacksRef<dyn Fn(Args) + Send + Sync, dyn FnMut(Args) + Send + Sync> {
    dyn_ref_impl!( + Send + Sync);
}

#[macro_export]
macro_rules! fn_alias {
    ($vis: vis type $name: ident = $ty: ident($($arg: ty),*)$($extra: tt)*) => {
        $vis type $name = $ty<dyn Fn($($arg,)*)$($extra)*, dyn FnMut($($arg,)*)$($extra)*>;
    };
}
