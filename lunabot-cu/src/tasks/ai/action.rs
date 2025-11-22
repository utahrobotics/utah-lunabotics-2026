use common::{LunabotStage, Steering};
use embedded_common::ActuatorCommand;

#[derive(Clone, Debug)]
pub enum LunabotAction {
    Yield,
    SetSteering(Steering),
    SetLastSteering,
    SetLastLift,
    SetLastBucket,
    SetLift(i8),
    SetBucket(i8),
    // actions for checking the lunabot stage (dig, dump, manual, soft stop, navigate)
    IsSoftStop,
    IsAutonomy,
    IsManual,
    None,

    // autonomy related things
    IsObstacleMapReady,

    /// if the robot is in an occupied cell, we should first pathfind to the nearest free cell (if a free cell is within some range)
    IsInOccupiedCell,
    IsInFreeCell,
    /// Success if the robot has reached destination, Running if the robot is currently navigating, Failiure if the robot got stuck
    CheckNavigation,
    /// if the robot is in a cell of unknown status, we should first pathfind to the nearest free cell (if a free cell is within some range)
    IsInUnknownCell,
    /// calculates path from the robots position to x,y
    CalculatePath,
    FollowPath,
    SetStage(LunabotStage),
    GetUnstuck,
}
