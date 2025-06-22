use std::sync::Arc;

use crossbeam::queue::{ArrayQueue, SegQueue};
use tokio::sync::Notify;

use super::caller::try_drop_this_callback;

enum Queue<T> {
    Bounded(ArrayQueue<T>),
    Unbounded(SegQueue<T>),
}

impl<T> Queue<T> {
    /// Pushes the given value onto the queue.
    ///
    /// Returns the value if the queue is full. If the queue
    /// is unbounded, this will never return an error.
    #[inline]
    fn push(&self, value: T) -> Result<(), T> {
        match self {
            Self::Bounded(queue) => queue.push(value),
            Self::Unbounded(queue) => {
                queue.push(value);
                Ok(())
            }
        }
    }

    /// Forcefully pushes the given value onto the queue, returning
    /// the oldest value if the queue is full.
    ///
    /// If the queue is unbounded, this will always return `None`.
    #[inline]
    fn force_push(&self, value: T) -> Option<T> {
        match self {
            Self::Bounded(queue) => queue.force_push(value),
            Self::Unbounded(queue) => {
                queue.push(value);
                None
            }
        }
    }

    /// Returns the oldest value in the queue.
    #[inline]
    fn pop(&self) -> Option<T> {
        match self {
            Self::Bounded(queue) => queue.pop(),
            Self::Unbounded(queue) => queue.pop(),
        }
    }
}

struct SubscriberInner<T> {
    queue: Queue<T>,
    notify: Notify,
}

pub struct Subscriber<T> {
    inner: Arc<SubscriberInner<T>>,
}

impl<T> Subscriber<T> {
    /// Creates a new subscriber with the given maximum size.
    pub fn new(max_size: usize) -> Self {
        Self {
            inner: Arc::new(SubscriberInner {
                queue: Queue::Bounded(ArrayQueue::new(max_size)),
                notify: Notify::new(),
            }),
        }
    }

    /// Creates a new subscriber that has no maximum size.
    pub fn new_unbounded() -> Self {
        Self {
            inner: Arc::new(SubscriberInner {
                queue: Queue::Unbounded(SegQueue::new()),
                notify: Notify::new(),
            }),
        }
    }

    /// Try to receive a value, returning `None` if no values are available.
    #[inline]
    pub fn try_recv(&self) -> Option<T> {
        self.inner.queue.pop()
    }

    /// Returns `true` if all callbacks that were made were dropped.
    #[inline]
    pub fn is_closed(&self) -> bool {
        Arc::weak_count(&self.inner) == 0
    }

    /// Receives a value, blocking until a value is available, or
    /// returning `None` if the subscriber is closed.
    pub async fn recv(&self) -> Option<T> {
        loop {
            if let Some(value) = self.inner.queue.pop() {
                return Some(value);
            }

            if self.is_closed() {
                return None;
            }

            self.inner.notify.notified().await;
        }
    }

    /// Receives a value, blocking until a value is available, or
    /// blocking forever if the subscriber is closed.
    ///
    /// # Note
    /// This will still await forever even if during awaiting, a callback
    /// is made.
    pub async fn recv_or_never(&self) -> T {
        if let Some(value) = self.recv().await {
            value
        } else {
            std::future::pending().await
        }
    }

    pub fn put(&self, value: T) {
        if self.inner.queue.force_push(value).is_none() {
            self.inner.notify.notify_one();
        }
    }

    pub fn put_conservative(&self, value: T) {
        if self.inner.queue.push(value).is_ok() {
            self.inner.notify.notify_one();
        }
    }

    /// Creates a callback that will add given values to this `Subscriber`.
    ///
    /// If the `Subscriber` is full, the given value is dropped immediately.
    pub fn create_conservative_callback(&self) -> impl Fn(T) + Send + Sync
    where
        T: Send,
    {
        let inner = Arc::downgrade(&self.inner.clone());
        move |value| {
            let Some(inner) = inner.upgrade() else {
                try_drop_this_callback();
                return;
            };
            if inner.queue.push(value).is_ok() {
                inner.notify.notify_one();
            }
        }
    }

    /// Creates a callback that will add given values to this `Subscriber`.
    ///
    /// If the `Subscriber` is full, the oldest value in the `Subscriber` is dropped.
    pub fn create_callback(&self) -> impl Fn(T) + Send + Sync
    where
        T: Send,
    {
        let inner = Arc::downgrade(&self.inner.clone());
        move |value| {
            let Some(inner) = inner.upgrade() else {
                try_drop_this_callback();
                return;
            };
            if inner.queue.force_push(value).is_none() {
                inner.notify.notify_one();
            }
        }
    }
}
