use common::Steering;
use embedded_common::ActuatorCommand;

#[derive(Clone, Debug)]
pub enum LunabotAction {
    SetSteering(Steering),
    SetLift(i8),
    SetBucket(i8),
    /// actions for checking the lunabot stage (dig, dump, manual, soft stop, navigate)
    IsSoftStop,
    IsAutonomy,
    IsManual,
    None,
}
