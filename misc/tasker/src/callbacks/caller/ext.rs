use std::marker::Tuple;

use super::CallbacksStorage;

macro_rules! impl_ext {
    ($name: ident $($extra: tt)*) => {
        pub trait $name {
            type Args: Tuple;
            fn add_fn(&mut self, callback: impl Fn<Self::Args, Output=()>$($extra)* + 'static);
            fn add_fn_mut(&mut self, callback: impl FnMut<Self::Args, Output=()>$($extra)* + 'static);
            fn add_fn_immut(&self, callback: impl Fn<Self::Args, Output=()>$($extra)* + 'static);
            fn add_fn_mut_immut(&self, callback: impl FnMut<Self::Args, Output=()>$($extra)* + 'static);
        }


        impl<Args: Tuple, T: CallbacksStorage<Immut = dyn Fn<Args, Output=()>, Mut=dyn FnMut<Args, Output = ()>>> $name for T {
            type Args = Args;

            fn add_fn(&mut self, callback: impl Fn<Args, Output=()>$($extra)* + 'static) {
                self.add_dyn_fn(Box::new(callback));
            }

            fn add_fn_mut(&mut self, callback: impl FnMut<Args, Output=()>$($extra)* + 'static) {
                self.add_dyn_fn_mut(Box::new(callback));
            }

            fn add_fn_immut(&self, callback: impl Fn<Args, Output=()>$($extra)* + 'static) {
                self.add_dyn_fn_immut(Box::new(callback));
            }

            fn add_fn_mut_immut(&self, callback: impl FnMut<Args, Output=()>$($extra)* + 'static) {
                self.add_dyn_fn_mut_immut(Box::new(callback));
            }
        }
    }
}

impl_ext!(CallbacksStorageExt + Send + Sync);
impl_ext!(CallbacksStorageUnsyncExt + Send);
impl_ext!(CallbacksStorageUnsendExt + Sync);
impl_ext!(CallbacksStorageUnsendUnsyncExt);
