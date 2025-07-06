use std::time::Duration;

use common::{FromAI, Steering};
use embedded_common::{Actuator, ActuatorCommand};
use nalgebra::Vector2;

use crate::context::HostHandle;

use super::{move_actuators, SoftStopped};

pub async fn dig_dump(host_handle: &mut HostHandle, dump_coords: Vector2<f64>) -> SoftStopped {
    host_handle.write_to_host(FromAI::SetStage(common::LunabotStage::Autonomy));
    host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
    if move_actuators(host_handle, Some(1000), Some(2000)).await.called {
        return SoftStopped { called: true };
    }
    tokio::select! {
        _ = tokio::time::sleep(Duration::from_millis(3000)) => {}
        _ = async {
            loop {
                host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(0.8, Actuator::Lift)));
                host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(0.8, Actuator::Bucket)));
                host_handle.write_to_host(FromAI::SetSteering(Steering::new(1.0, 1.0, Steering::DEFAULT_WEIGHT)));
                tokio::time::sleep(Duration::from_millis(80)).await;
            }
        } => {}
    }
    host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
    if move_actuators(host_handle, Some(2200), None).await.called {
        return SoftStopped { called: true };
    }
    eprintln!("Navigational autonomy not implemented");
    if move_actuators(host_handle, Some(1800), Some(3000)).await.called {
        return SoftStopped { called: true };
    }
    SoftStopped { called: false }
}