use crate::{
    Behavior, CancelSafe, EternalBehavior, EternalStatus, FallibleBehavior, FallibleStatus,
    InfallibleBehavior, InfallibleStatus, IntoRon, Status,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum IfElseState {
    Condition,
    IfTrue,
    IfFalse,
}

pub struct IfElse<A, B, C> {
    pub condition: A,
    pub if_true: B,
    pub if_false: C,
    state: IfElseState,
}

impl<A, B, C> IntoRon for IfElse<A, B, C>
where
    A: IntoRon,
    B: IntoRon,
    C: IntoRon,
{
    fn into_ron(&self) -> ron::Value {
        ron::Value::Map(
            [
                (
                    ron::Value::String("condition".to_string()),
                    self.condition.into_ron(),
                ),
                (
                    ron::Value::String("success".to_string()),
                    self.if_true.into_ron(),
                ),
                (
                    ron::Value::String("failure".to_string()),
                    self.if_false.into_ron(),
                ),
            ]
            .into_iter()
            .collect(),
        )
    }
}

impl<A, B, C, D> Behavior<D> for IfElse<A, B, C>
where
    A: Behavior<D>,
    B: Behavior<D>,
    C: Behavior<D>,
{
    fn run(&mut self, blackboard: &mut D) -> Status {
        let result = match self.state {
            IfElseState::Condition => match self.condition.run(blackboard) {
                Status::Running => return Status::Running,
                Status::Success => {
                    self.state = IfElseState::IfTrue;
                    self.if_true.run(blackboard)
                }
                Status::Failure => {
                    self.state = IfElseState::IfFalse;
                    self.if_false.run(blackboard)
                }
            },
            IfElseState::IfTrue => self.if_true.run(blackboard),
            IfElseState::IfFalse => self.if_false.run(blackboard),
        };

        if !result.is_running() {
            self.state = IfElseState::Condition;
        }

        result
    }
}

impl<A, B, C> CancelSafe for IfElse<A, B, C>
where
    A: CancelSafe,
    B: CancelSafe,
    C: CancelSafe,
{
    fn reset(&mut self) {
        self.state = IfElseState::Condition;
        self.condition.reset();
        self.if_true.reset();
        self.if_false.reset();
    }
}

impl<A, B, C, D> InfallibleBehavior<D> for IfElse<A, B, C>
where
    A: Behavior<D>,
    B: InfallibleBehavior<D>,
    C: InfallibleBehavior<D>,
{
    fn run_infallible(&mut self, blackboard: &mut D) -> InfallibleStatus {
        let result = match self.state {
            IfElseState::Condition => match self.condition.run(blackboard) {
                Status::Running => return InfallibleStatus::Running,
                Status::Success => {
                    self.state = IfElseState::IfTrue;
                    self.if_true.run_infallible(blackboard)
                }
                Status::Failure => {
                    self.state = IfElseState::IfFalse;
                    self.if_false.run_infallible(blackboard)
                }
            },
            IfElseState::IfTrue => self.if_true.run_infallible(blackboard),
            IfElseState::IfFalse => self.if_false.run_infallible(blackboard),
        };

        if !result.is_running() {
            self.state = IfElseState::Condition;
        }

        result
    }
}

impl<A, B, C, D> FallibleBehavior<D> for IfElse<A, B, C>
where
    A: Behavior<D>,
    B: FallibleBehavior<D>,
    C: FallibleBehavior<D>,
{
    fn run_fallible(&mut self, blackboard: &mut D) -> FallibleStatus {
        let result = match self.state {
            IfElseState::Condition => match self.condition.run(blackboard) {
                Status::Running => return FallibleStatus::Running,
                Status::Success => {
                    self.state = IfElseState::IfTrue;
                    self.if_true.run_fallible(blackboard)
                }
                Status::Failure => {
                    self.state = IfElseState::IfFalse;
                    self.if_false.run_fallible(blackboard)
                }
            },
            IfElseState::IfTrue => self.if_true.run_fallible(blackboard),
            IfElseState::IfFalse => self.if_false.run_fallible(blackboard),
        };

        if !result.is_running() {
            self.state = IfElseState::Condition;
        }

        result
    }
}

impl<A, B, C, D> EternalBehavior<D> for IfElse<A, B, C>
where
    A: Behavior<D>,
    B: EternalBehavior<D>,
    C: EternalBehavior<D>,
{
    fn run_eternal(&mut self, blackboard: &mut D) -> EternalStatus {
        match self.state {
            IfElseState::Condition => match self.condition.run(blackboard) {
                Status::Running => return EternalStatus::Running,
                Status::Success => {
                    self.state = IfElseState::IfTrue;
                    self.if_true.run_eternal(blackboard)
                }
                Status::Failure => {
                    self.state = IfElseState::IfFalse;
                    self.if_false.run_eternal(blackboard)
                }
            },
            IfElseState::IfTrue => self.if_true.run_eternal(blackboard),
            IfElseState::IfFalse => self.if_false.run_eternal(blackboard),
        }
    }
}

impl<A, B, C> IfElse<A, B, C> {
    pub fn new(condition: A, if_true: B, if_false: C) -> Self {
        Self {
            condition,
            if_true,
            if_false,
            state: IfElseState::Condition,
        }
    }
}

pub struct TryCatch<A, B> {
    pub try_behavior: A,
    pub catch: B,
    trying: bool,
}

impl<A, B> IntoRon for TryCatch<A, B>
where
    A: IntoRon,
    B: IntoRon,
{
    fn into_ron(&self) -> ron::Value {
        ron::Value::Map(
            [
                (
                    ron::Value::String("try".to_string()),
                    self.try_behavior.into_ron(),
                ),
                (
                    ron::Value::String("catch".to_string()),
                    self.catch.into_ron(),
                ),
            ]
            .into_iter()
            .collect(),
        )
    }
}

impl<A, B, D> Behavior<D> for TryCatch<A, B>
where
    A: Behavior<D>,
    B: Behavior<D>,
{
    fn run(&mut self, blackboard: &mut D) -> Status {
        let result = if self.trying {
            match self.try_behavior.run(blackboard) {
                Status::Running => return Status::Running,
                Status::Success => return Status::Success,
                Status::Failure => {
                    self.trying = false;
                    self.catch.run(blackboard)
                }
            }
        } else {
            self.catch.run(blackboard)
        };

        if !result.is_running() {
            self.trying = true;
        }

        result
    }
}

impl<A, B> CancelSafe for TryCatch<A, B>
where
    A: CancelSafe,
    B: CancelSafe,
{
    fn reset(&mut self) {
        self.trying = true;
        self.try_behavior.reset();
        self.catch.reset();
    }
}

impl<A, B, D> InfallibleBehavior<D> for TryCatch<A, B>
where
    A: Behavior<D>,
    B: InfallibleBehavior<D>,
{
    fn run_infallible(&mut self, blackboard: &mut D) -> InfallibleStatus {
        let result = if self.trying {
            match self.try_behavior.run(blackboard) {
                Status::Running => return InfallibleStatus::Running,
                Status::Success => return InfallibleStatus::Success,
                Status::Failure => {
                    self.trying = false;
                    self.catch.run_infallible(blackboard)
                }
            }
        } else {
            self.catch.run_infallible(blackboard)
        };

        if !result.is_running() {
            self.trying = true;
        }

        result
    }
}

impl<A, B, D> FallibleBehavior<D> for TryCatch<A, B>
where
    A: FallibleBehavior<D>,
    B: FallibleBehavior<D>,
{
    fn run_fallible(&mut self, blackboard: &mut D) -> FallibleStatus {
        let result = if self.trying {
            match self.try_behavior.run_fallible(blackboard) {
                FallibleStatus::Running => return FallibleStatus::Running,
                FallibleStatus::Failure => {
                    self.trying = false;
                    self.catch.run_fallible(blackboard)
                }
            }
        } else {
            self.catch.run_fallible(blackboard)
        };

        if !result.is_running() {
            self.trying = true;
        }

        result
    }
}

impl<A, B, D> EternalBehavior<D> for TryCatch<A, B>
where
    A: EternalBehavior<D>,
    B: Behavior<D>,
{
    fn run_eternal(&mut self, blackboard: &mut D) -> EternalStatus {
        self.trying = true;
        self.try_behavior.run_eternal(blackboard)
    }
}

impl<A, B> TryCatch<A, B> {
    pub fn new(try_behavior: A, catch: B) -> Self {
        Self {
            try_behavior,
            catch,
            trying: true,
        }
    }
}
