use common::{FromAI, FromHost, FromLunabase, LunabotStage, Steering};
use embedded_common::{Actuator, ActuatorCommand};

use crate::context::HostHandle;

pub async fn software_stop(host_handle: &mut HostHandle) {
    host_handle.write_to_host(FromAI::SetStage(LunabotStage::SoftStop));
    host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
    host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(0.0, Actuator::Bucket)));
    host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(0.0, Actuator::Lift)));

    loop {
        let msg = host_handle.read_from_host().await;
        let FromHost::FromLunabase { msg } = msg else {
            continue;
        };
        match msg { 
            FromLunabase::ContinueMission => break,
            _ => {}
        }
    }
}