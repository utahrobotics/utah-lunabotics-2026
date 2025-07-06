#![feature(unboxed_closures)]
#![feature(tuple_trait)]
#![feature(never_type)]
#![feature(get_mut_unchecked)]

use std::{
    backtrace::Backtrace,
    cell::RefCell,
    future::Future,
    num::NonZeroUsize,
    ops::Deref,
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc,
    },
    time::Duration,
};

use crossbeam::queue::SegQueue;
pub use parking_lot;
use parking_lot::{RwLock, RwLockReadGuard, RwLockWriteGuard};
pub use tokio;
use tokio::{runtime::Handle, sync::Notify};

use cu29::prelude::*;

pub mod callbacks;
pub mod shared;
pub mod task;

#[derive(Clone, Copy)]
pub struct TokioRuntimeConfig {
    pub num_threads: Option<NonZeroUsize>,
    pub shutdown_delayed_warning: Duration,
    pub thread_delayed_warning: Duration,
}

impl Default for TokioRuntimeConfig {
    fn default() -> Self {
        Self {
            num_threads: None,
            shutdown_delayed_warning: Duration::from_secs(5),
            thread_delayed_warning: Duration::from_secs(5),
        }
    }
}

impl TokioRuntimeConfig {
    pub fn build(self) {
        with_tokio_runtime_handle(|_| {}, || self);
    }
}

struct TokioRuntimeHandle {
    handle: Handle,
    // runtime_ended: Arc<NotifyFlag>,
    end_runtime: Arc<NotifyFlag>,
    attached_guards: Arc<SegQueue<Arc<RuntimeDropGuardInner>>>,
    index: usize,
}

static RUNTIME_INDEX: AtomicUsize = AtomicUsize::new(0);
static TOKIO_RUNTIME_HANDLE: RwLock<Option<TokioRuntimeHandle>> = RwLock::new(None);

fn with_tokio_runtime_handle<T>(
    f: impl FnOnce(&TokioRuntimeHandle) -> T,
    config: impl FnOnce() -> TokioRuntimeConfig,
) -> T {
    {
        let reader = TOKIO_RUNTIME_HANDLE.read();
        if let Some(handle) = reader.as_ref() {
            return f(handle);
        }
    }
    let mut writer = TOKIO_RUNTIME_HANDLE.write();
    if writer.is_some() {
        f(RwLockWriteGuard::downgrade(writer)
            .deref()
            .as_ref()
            .unwrap())
    } else {
        let config = config();
        let mut builder = tokio::runtime::Builder::new_multi_thread();

        if let Some(num_threads) = config.num_threads {
            builder.worker_threads(num_threads.get());
        }

        let runtime = builder.enable_all().build().unwrap();

        let handle = runtime.handle().clone();
        let runtime_ended: Arc<NotifyFlag> = Arc::default();
        let end_runtime: Arc<NotifyFlag> = Arc::default();
        let attached_guards: Arc<SegQueue<Arc<RuntimeDropGuardInner>>> = Arc::default();
        let tokio_runtime_handle = TokioRuntimeHandle {
            handle,
            // runtime_ended: runtime_ended.clone(),
            end_runtime: end_runtime.clone(),
            attached_guards: attached_guards.clone(),
            index: RUNTIME_INDEX.fetch_add(1, Ordering::Acquire),
        };
        writer.replace(tokio_runtime_handle);

        std::thread::spawn(move || {
            runtime.block_on(async {
                end_runtime.notified().await;
                let mut attached_guards: Vec<RuntimeDropGuardInner> = Vec::with_capacity(attached_guards.len());
                while let Some(guard) = attached_guards.pop() {
                    attached_guards.push(guard);
                }
                tokio::select! {
                    () = async {
                        for drop_notify in &attached_guards {
                            drop_notify.notify.notified().await;
                        }
                    } => {}
                    () = tokio::time::sleep(config.thread_delayed_warning) => {
                        let mut backtraces = String::new();
                        for drop_notify in &attached_guards {
                            backtraces.push_str(&format!("{:?}\n\n", drop_notify.backtrace));
                        }
                        warning!(
                            "The following guards have not dropped after {:.1} seconds\n\n{}",
                            config.thread_delayed_warning.as_secs_f32()
                        );
                        for drop_notify in &attached_guards {
                            drop_notify.notify.notified().await;
                        }
                    }
                }
            });
            let runtime_ended2 = runtime_ended.clone();
            std::thread::spawn(move || {
                std::thread::sleep(config.shutdown_delayed_warning);
                if !runtime_ended2.notified.load(Ordering::Acquire) {
                    warning!(
                        "Tokio runtime has not ended after {:.1} seconds",
                        config.shutdown_delayed_warning.as_secs_f32()
                    );
                }
            });
            drop(runtime);
            TOKIO_RUNTIME_HANDLE.write().take();
            runtime_ended.notify();
        });

        f(RwLockWriteGuard::downgrade(writer)
            .deref()
            .as_ref()
            .unwrap())
    }
}

/// Gets a `Handle` to the active tokio runtime.
///
/// If one does not exist, a new tokio runtime will be created using `TokioRuntimeConfig`.
/// See `set_tokio_runtime_config`.
#[inline(always)]
pub fn get_tokio_handle() -> Handle {
    with_tokio_runtime_handle(|handle| handle.handle.clone(), Default::default)
}

pub fn end_tokio_runtime() {
    if let Some(handle) = TOKIO_RUNTIME_HANDLE.read().as_ref() {
        handle.end_runtime.notify();
    }
}

pub fn end_tokio_runtime_and_wait() {
    let mut reader = TOKIO_RUNTIME_HANDLE.read();
    if let Some(handle) = reader.as_ref() {
        let index = handle.index;
        handle.end_runtime.notify();

        loop {
            RwLockReadGuard::bump(&mut reader);
            if let Some(handle) = reader.as_ref() {
                if handle.index == index {
                    continue;
                }
            }
            break;
        }
    }
}

/// Returns true iff there is no tokio runtime.
pub fn has_tokio_runtime_ended() -> bool {
    TOKIO_RUNTIME_HANDLE.read().is_none()
}

/// Returns true iff there is a tokio runtime and it is ending.
pub fn is_tokio_runtime_ending() -> bool {
    if let Some(handle) = TOKIO_RUNTIME_HANDLE.read().as_ref() {
        handle.end_runtime.was_notified()
    } else {
        false
    }
}

// /// Asynchronously waits for the tokio runtime to end if it exists.
// ///
// /// Blocking on this future using `BlockOn` is strongly discouraged as the runtime
// /// used to poll this future will be the same runtime that is ending.
// pub async fn await_for_tokio_runtime_end() {
//     if let Some(handle) = TOKIO_RUNTIME_HANDLE.read().as_ref() {
//         handle.runtime_ended.notified().await;
//     }
// }

pub trait BlockOn: Future {
    fn block_on(self) -> Self::Output;
}

impl<T> BlockOn for T
where
    T: Future,
{
    #[inline(always)]
    fn block_on(self) -> Self::Output {
        get_tokio_handle().block_on(self)
    }
}

#[derive(Default)]
struct NotifyFlag {
    notified: AtomicBool,
    notify: Notify,
}

impl NotifyFlag {
    // const fn new() -> Self {
    //     Self {
    //         notified: AtomicBool::new(false),
    //         notify: Notify::const_new(),
    //     }
    // }

    async fn notified(&self) {
        let notified = self.notify.notified();
        if self.was_notified() {
            return;
        }
        notified.await;
    }

    fn was_notified(&self) -> bool {
        self.notified.load(Ordering::Acquire)
    }

    fn notify(&self) {
        self.notified.store(true, Ordering::Release);
        self.notify.notify_waiters();
    }
}

struct RuntimeDropGuardInner {
    backtrace: Backtrace,
    notify: NotifyFlag,
}

/// A guard that prevents the Tokio Runtime from exiting.
///
/// For the guard to work, if must be instantiated *after* the runtime has been created.
/// Creating the runtime after the guard will result in the guard not doing anything. You
/// can enforce creation of the tokio runtime using `get_tokio_handle`.
///
/// Refer to 'attach_drop_guard' and 'detach_drop_guard' for a more convenient way to manage
/// drop guards.
pub struct RuntimeDropGuard(Arc<RuntimeDropGuardInner>);

impl Default for RuntimeDropGuard {
    fn default() -> Self {
        let inner = Arc::new(RuntimeDropGuardInner {
            backtrace: Backtrace::force_capture(),
            notify: Default::default(),
        });
        if let Some(handle) = TOKIO_RUNTIME_HANDLE.read().as_ref() {
            handle.attached_guards.push(inner.clone());
        }
        RuntimeDropGuard(inner)
    }
}

impl RuntimeDropGuard {
    /// Returns `true` iff this guard will prevent the tokio runtime from exiting.
    pub fn is_attached(&self) -> bool {
        Arc::strong_count(&self.0) > 1
    }
}

impl Drop for RuntimeDropGuard {
    fn drop(&mut self) {
        self.0.notify.notify();
    }
}

thread_local! {
    static DROP_NOTIFY: RefCell<Option<RuntimeDropGuard>> = const { RefCell::new(None) };
}

/// Creates a `RuntimeDropGuard` that is stored in thread local data, which eseentially
/// prevents the tokio runtime from exiting until the current thread exits.
pub fn attach_drop_guard() {
    DROP_NOTIFY.with_borrow_mut(|x| *x = Some(RuntimeDropGuard::default()));
}

/// Detaches the `RuntimeDropGuard` created by `attach_drop_guard` and returns it.
///
/// This is the recommended way to drop the guard instead of relying on the thread
/// to drop it as there are several caveats related to a thread's ability to drop
/// thread local data. If the thread fails to drop the guard, the runtime will never
/// exit.
pub fn detach_drop_guard() -> Option<RuntimeDropGuard> {
    DROP_NOTIFY.with_borrow_mut(Option::take)
}

#[macro_export]
macro_rules! duration_warning {
    ($inner: block within $duration: expr) => {{
        let start = std::time::Instant::now();
        let result = $inner;
        let duration = start.elapsed();
        if duration > Duration::from_secs(1) {
            $crate::tracing::warn!(
                "{} took {:.1} seconds",
                stringify!($inner),
                duration.as_secs_f32()
            );
        }
        result
    }};
}
