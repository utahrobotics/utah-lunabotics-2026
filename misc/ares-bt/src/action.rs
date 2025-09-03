use crate::{
    Behavior, CancelSafe, EternalBehavior, EternalStatus, FallibleBehavior, FallibleStatus,
    InfallibleBehavior, InfallibleStatus, IntoRon, Status,
};

impl<F: FnMut(&mut B) -> Status, B> Behavior<B> for F {
    fn run(&mut self, blackboard: &mut B) -> Status {
        self(blackboard)
    }
}

impl<F: FnMut(&mut B) -> InfallibleStatus, B> InfallibleBehavior<B> for F {
    fn run_infallible(&mut self, blackboard: &mut B) -> InfallibleStatus {
        self(blackboard)
    }
}

impl<F: FnMut(&mut B) -> FallibleStatus, B> FallibleBehavior<B> for F {
    fn run_fallible(&mut self, blackboard: &mut B) -> FallibleStatus {
        self(blackboard)
    }
}

impl<F: FnMut(&mut B) -> EternalStatus, B> EternalBehavior<B> for F {
    fn run_eternal(&mut self, blackboard: &mut B) -> EternalStatus {
        self(blackboard)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AlwaysSucceed;

impl<B> Behavior<B> for AlwaysSucceed {
    fn run(&mut self, _blackboard: &mut B) -> Status {
        Status::Success
    }
}

impl<B> InfallibleBehavior<B> for AlwaysSucceed {
    fn run_infallible(&mut self, _blackboard: &mut B) -> InfallibleStatus {
        InfallibleStatus::Success
    }
}

impl IntoRon for AlwaysSucceed {
    fn into_ron(&self) -> ron::Value {
        ron::Value::String("AlwaysSucceed".to_string())
    }
}

impl CancelSafe for AlwaysSucceed {
    fn reset(&mut self) {}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AlwaysFail;

impl<B> Behavior<B> for AlwaysFail {
    fn run(&mut self, _blackboard: &mut B) -> Status {
        Status::Failure
    }
}

impl<B> FallibleBehavior<B> for AlwaysFail {
    fn run_fallible(&mut self, _blackboard: &mut B) -> FallibleStatus {
        FallibleStatus::Failure
    }
}

impl IntoRon for AlwaysFail {
    fn into_ron(&self) -> ron::Value {
        ron::Value::String("AlwaysFail".to_string())
    }
}

impl CancelSafe for AlwaysFail {
    fn reset(&mut self) {}
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AlwaysRunning;

impl<B> Behavior<B> for AlwaysRunning {
    fn run(&mut self, _blackboard: &mut B) -> Status {
        Status::Running
    }
}

impl<B> InfallibleBehavior<B> for AlwaysRunning {
    fn run_infallible(&mut self, _blackboard: &mut B) -> InfallibleStatus {
        InfallibleStatus::Running
    }
}

impl<B> FallibleBehavior<B> for AlwaysRunning {
    fn run_fallible(&mut self, _blackboard: &mut B) -> FallibleStatus {
        FallibleStatus::Running
    }
}

impl<B> EternalBehavior<B> for AlwaysRunning {
    fn run_eternal(&mut self, _blackboard: &mut B) -> EternalStatus {
        Default::default()
    }
}

impl CancelSafe for AlwaysRunning {
    fn reset(&mut self) {}
}

impl IntoRon for AlwaysRunning {
    fn into_ron(&self) -> ron::Value {
        ron::Value::String("AlwaysRunning".to_string())
    }
}

// pub struct RunOnce<F> {
//     pub func: F,
//     ran: bool,
// }

// impl<F> From<F> for RunOnce<F> {
//     fn from(func: F) -> Self {
//         Self { func, ran: false }
//     }
// }

// impl<B, F: FnMut() -> T> Behavior<B> for RunOnce<F> {
//     fn run(&mut self, _blackboard: &mut B) -> Status {
//         if self.ran {
//             self.ran = false;
//             Status::Success
//         } else {
//             self.ran = true;
//             Status::Running
//         }
//     }
// }

// impl<B, F: FnMut() -> T> InfallibleBehavior<B> for RunOnce<F> {
//     fn run_infallible(&mut self, _blackboard: &mut B) -> InfallibleStatus {
//         if self.ran {
//             self.ran = false;
//             InfallibleStatus::Success
//         } else {
//             self.ran = true;
//             InfallibleStatus::Running
//         }
//     }
// }

// impl<F> CancelSafe for RunOnce<F> {
//     fn reset(&mut self) {
//         self.ran = false;
//     }
// }
