use cu29::prelude::*;

#[deprecated]
pub trait SyncTask: Send + Sized + 'static {
    type Output;

    fn run(self) -> Self::Output;
    fn spawn(self) -> std::thread::JoinHandle<()>
    where
        Self::Output: Loggable,
    {
        self.spawn_with(|x| x.log())
    }
    fn spawn_with(
        self,
        f: impl FnOnce(Self::Output) + Send + 'static,
    ) -> std::thread::JoinHandle<()> {
        std::thread::spawn(move || {
            f(self.run());
        })
    }
}

pub trait Loggable {
    fn log(&self);
}

impl Loggable for () {
    fn log(&self) {}
}

impl Loggable for ! {
    fn log(&self) {}
}

impl Loggable for String {
    fn log(&self) {
        info!("{self}");
    }
}

impl Loggable for &'static str {
    fn log(&self) {
        info!("{self}");
    }
}

impl<T: Loggable, E: std::fmt::Display> Loggable for Result<T, E> {
    fn log(&self) {
        match self {
            Ok(t) => t.log(),
            Err(e) => error!("{e}"),
        }
    }
}
