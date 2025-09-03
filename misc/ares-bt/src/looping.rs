use crate::{
    Behavior, CancelSafe, EternalBehavior, EternalStatus, FallibleBehavior, FallibleStatus,
    InfallibleBehavior, InfallibleStatus, IntoRon, Status,
};

pub struct WhileLoop<A, B> {
    pub condition: A,
    pub body: B,
    check_condition: bool,
}

impl<A, B> IntoRon for WhileLoop<A, B>
where
    A: IntoRon,
    B: IntoRon,
{
    fn into_ron(&self) -> ron::Value {
        ron::Value::Map(
            [
                (
                    ron::Value::String("condition".to_string()),
                    self.condition.into_ron(),
                ),
                (ron::Value::String("body".to_string()), self.body.into_ron()),
            ]
            .into_iter()
            .collect(),
        )
    }
}

impl<A, B, D> Behavior<D> for WhileLoop<A, B>
where
    A: Behavior<D>,
    B: Behavior<D>,
{
    fn run(&mut self, blackboard: &mut D) -> Status {
        loop {
            if self.check_condition {
                match self.condition.run(blackboard) {
                    Status::Running => return Status::Running,
                    Status::Success => self.check_condition = false,
                    Status::Failure => return Status::Success,
                }
            }
            match self.body.run(blackboard) {
                Status::Running => return Status::Running,
                Status::Success => self.check_condition = true,
                Status::Failure => {
                    self.check_condition = true;
                    return Status::Failure;
                }
            }
        }
    }
}

impl<A, B> CancelSafe for WhileLoop<A, B>
where
    A: CancelSafe,
    B: CancelSafe,
{
    fn reset(&mut self) {
        self.check_condition = true;
        self.condition.reset();
        self.body.reset();
    }
}

impl<A, B, D> FallibleBehavior<D> for WhileLoop<A, B>
where
    A: InfallibleBehavior<D>,
    B: FallibleBehavior<D>,
{
    fn run_fallible(&mut self, blackboard: &mut D) -> FallibleStatus {
        loop {
            if self.check_condition {
                match self.condition.run_infallible(blackboard) {
                    InfallibleStatus::Running => return FallibleStatus::Running,
                    InfallibleStatus::Success => self.check_condition = false,
                }
            }
            match self.body.run_fallible(blackboard) {
                FallibleStatus::Running => return FallibleStatus::Running,
                FallibleStatus::Failure => {
                    self.check_condition = true;
                    return FallibleStatus::Failure;
                }
            }
        }
    }
}

impl<A, B, D> InfallibleBehavior<D> for WhileLoop<A, B>
where
    A: Behavior<D>,
    B: InfallibleBehavior<D>,
{
    fn run_infallible(&mut self, blackboard: &mut D) -> InfallibleStatus {
        loop {
            if self.check_condition {
                match self.condition.run(blackboard) {
                    Status::Running => return InfallibleStatus::Running,
                    Status::Success => self.check_condition = false,
                    Status::Failure => return InfallibleStatus::Success,
                }
            }
            match self.body.run_infallible(blackboard) {
                InfallibleStatus::Running => return InfallibleStatus::Running,
                InfallibleStatus::Success => self.check_condition = true,
            }
        }
    }
}

impl<A, B, D> EternalBehavior<D> for WhileLoop<A, B>
where
    A: InfallibleBehavior<D>,
    B: InfallibleBehavior<D>,
{
    fn run_eternal(&mut self, blackboard: &mut D) -> EternalStatus {
        loop {
            if self.check_condition {
                match self.condition.run_infallible(blackboard) {
                    InfallibleStatus::Running => return EternalStatus::Running,
                    InfallibleStatus::Success => self.check_condition = false,
                }
            }
            match self.body.run_infallible(blackboard) {
                InfallibleStatus::Running => return EternalStatus::Running,
                InfallibleStatus::Success => self.check_condition = true,
            }
        }
    }
}

impl<A, B> WhileLoop<A, B> {
    pub fn new(condition: A, body: B) -> Self {
        Self {
            condition,
            body,
            check_condition: true,
        }
    }
}
