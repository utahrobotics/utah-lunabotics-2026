use common::Steering;
use embedded_common::ActuatorCommand;

#[derive(Clone, Debug)]
pub enum LunabotAction {
    SetSteering(Steering),
    SetActuators(ActuatorCommand),
    /// actions for checking the lunabot stage (dig, dump, manual, soft stop, navigate)
    IsSoftStop,
    IsAutonomy,
    IsTeleOp,
}
