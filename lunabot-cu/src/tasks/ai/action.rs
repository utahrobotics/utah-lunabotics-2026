use common::Steering;
use embedded_common::ActuatorCommand;

#[derive(Clone, Debug)]
pub enum LunabotAction {
    SetSteering(Steering),
    SetActuators(ActuatorCommand),
    /// actions for checking the lunabot stage (dig, dump, manual, soft stop, navigate)
    IsSoftStop,
    IsAutonomy,
    IsManual,

    /// Actions to check if there is a steering or actuator command waiting to be processed from the lunabase
    UseLastSteering,
    UseLastActuator,
}
