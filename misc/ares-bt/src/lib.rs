#![feature(unboxed_closures, fn_traits)]

use std::marker::PhantomData;

pub mod action;
pub mod branching;
pub mod converters;
pub mod looping;
pub mod sequence;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Status {
    Running,
    Success,
    Failure,
}

impl Status {
    pub const fn is_ok(self) -> bool {
        match self {
            Self::Running => false,
            Self::Success => true,
            Self::Failure => false,
        }
    }

    pub const fn is_err(self) -> bool {
        match self {
            Self::Running => false,
            Self::Success => false,
            Self::Failure => true,
        }
    }

    pub const fn is_running(self) -> bool {
        match self {
            Self::Running => true,
            Self::Success => false,
            Self::Failure => false,
        }
    }
}

impl From<bool> for Status {
    fn from(value: bool) -> Self {
        if value {
            Status::Success
        } else {
            Status::Failure
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FallibleStatus {
    Running,
    Failure,
}

impl FallibleStatus {
    pub const fn is_ok(self) -> bool {
        match self {
            Self::Running => false,
            Self::Failure => false,
        }
    }

    pub const fn is_err(self) -> bool {
        match self {
            Self::Running => false,
            Self::Failure => true,
        }
    }

    pub const fn is_running(self) -> bool {
        match self {
            Self::Running => true,
            Self::Failure => false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InfallibleStatus {
    Running,
    Success,
}

impl InfallibleStatus {
    pub const fn is_ok(self) -> bool {
        match self {
            Self::Running => false,
            Self::Success => true,
        }
    }

    pub const fn is_err(self) -> bool {
        match self {
            Self::Running => false,
            Self::Success => false,
        }
    }

    pub const fn is_running(self) -> bool {
        match self {
            Self::Running => true,
            Self::Success => false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum EternalStatus {
    #[default]
    Running,
}

impl EternalStatus {
    pub const fn is_ok(self) -> bool {
        match self {
            Self::Running => false,
        }
    }

    pub const fn is_err(self) -> bool {
        match self {
            Self::Running => false,
        }
    }

    pub const fn is_running(self) -> bool {
        match self {
            Self::Running => true,
        }
    }
}

/// A behavior that runs until it fails or succeeds.
pub trait Behavior<B> {
    fn run(&mut self, blackboard: &mut B) -> Status;
}

/// A behavior that runs until it succeeds.
pub trait InfallibleBehavior<B> {
    fn run_infallible(&mut self, blackboard: &mut B) -> InfallibleStatus;
}

/// A behavior that runs until it fails.
pub trait FallibleBehavior<B> {
    fn run_fallible(&mut self, blackboard: &mut B) -> FallibleStatus;
}

/// A behavior that runs forever.
pub trait EternalBehavior<B> {
    fn run_eternal(&mut self, blackboard: &mut B) -> EternalStatus;
}

pub trait IntoRon {
    fn into_ron(&self) -> ron::Value;
}
pub trait CancelSafe {
    fn reset(&mut self);
}

impl From<InfallibleStatus> for Status {
    fn from(value: InfallibleStatus) -> Self {
        match value {
            InfallibleStatus::Running => Status::Running,
            InfallibleStatus::Success => Status::Success,
        }
    }
}

impl From<FallibleStatus> for Status {
    fn from(value: FallibleStatus) -> Self {
        match value {
            FallibleStatus::Running => Status::Running,
            FallibleStatus::Failure => Status::Failure,
        }
    }
}

impl From<EternalStatus> for Status {
    fn from(value: EternalStatus) -> Self {
        match value {
            EternalStatus::Running => Status::Running,
        }
    }
}

pub struct RunningOnce<B> {
    ran_already: bool,
    phantom: PhantomData<fn() -> B>,
}

impl<B> Default for RunningOnce<B> {
    fn default() -> Self {
        Self {
            ran_already: false,
            phantom: PhantomData,
        }
    }
}

impl<B> Behavior<B> for RunningOnce<B> {
    fn run(&mut self, _blackboard: &mut B) -> Status {
        if self.ran_already {
            self.ran_already = false;
            Status::Success
        } else {
            self.ran_already = true;
            Status::Running
        }
    }
}

impl<B> CancelSafe for RunningOnce<B> {
    fn reset(&mut self) {
        self.ran_already = false;
    }
}

#[cfg(test)]
mod tests {
    use looping::WhileLoop;

    use super::*;

    #[test]
    fn test_sum() {
        let mut sum = 0;
        let is_ok = WhileLoop::new(
            |sum: &mut usize| (*sum < 10).into(),
            |sum: &mut usize| {
                *sum += 1;
                InfallibleStatus::Success
            },
        )
        .run_infallible(&mut sum)
        .is_ok();
        assert!(is_ok);
        assert_eq!(sum, 10);
    }
}
