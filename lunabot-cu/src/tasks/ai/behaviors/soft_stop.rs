use bonsai_bt::Behavior::{self, Action};
use common::Steering;
use embedded_common::{Actuator, ActuatorCommand};

use crate::tasks::ai::action::LunabotAction;

pub fn soft_stop_behavior() -> bonsai_bt::Behavior<crate::tasks::ai::action::LunabotAction> {
    Behavior::While(
        Box::new(Action(LunabotAction::IsSoftStop)),
        vec![
            Action(LunabotAction::SetSteering(Steering::new(0., 0.0, 0.0))),
            Action(LunabotAction::SetActuators(ActuatorCommand::SetSpeed(
                0,
                Actuator::Lift,
            ))),
            Action(LunabotAction::SetActuators(ActuatorCommand::SetSpeed(
                0,
                Actuator::Bucket,
            ))),
        ],
    )
}
