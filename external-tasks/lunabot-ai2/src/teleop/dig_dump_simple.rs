use std::time::Duration;

use common::{FromAI, FromHost, FromLunabase, Steering};

use crate::context::HostHandle;

use super::{move_actuators, SoftStopped};

pub async fn dig_dump_simple(host_handle: &mut HostHandle) -> SoftStopped {
    println!("Dig Dump");
    host_handle.write_to_host(FromAI::SetStage(common::LunabotStage::Autonomy));
    host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
    loop {
        if move_actuators(host_handle, Some(1000), Some(2000)).await.called {
            return SoftStopped { called: true };
        }
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_millis(1000)) => {}
            _ = async {
                loop {
                    println!("Driving");
                    host_handle.write_to_host(FromAI::SetSteering(Steering::new(1.0, 1.0, Steering::DEFAULT_WEIGHT)));
                    if let Some(FromHost::FromLunabase { msg: FromLunabase::SoftStop }) = host_handle.try_read_from_host() {
                        return SoftStopped { called: true };
                    }
                    tokio::time::sleep(Duration::from_millis(80)).await;
                }
            } => {}
        }
        host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
        if move_actuators(host_handle, Some(1800), Some(300)).await.called {
            return SoftStopped { called: true };
        }
        tokio::select! {
            _ = tokio::time::sleep(Duration::from_millis(3000)) => {}
            _ = async {
                loop {
                    host_handle.write_to_host(FromAI::SetSteering(Steering::new(0.5, 0.5, Steering::DEFAULT_WEIGHT)));
                    if let Some(FromHost::FromLunabase { msg: FromLunabase::SoftStop }) = host_handle.try_read_from_host() {
                        return SoftStopped { called: true };
                    }
                    tokio::time::sleep(Duration::from_millis(80)).await;
                }
            } => {}
        }
        
        if move_actuators(host_handle, None, Some(3000)).await.called {
            return SoftStopped { called: true };
        }

        tokio::time::sleep(Duration::from_millis(1000)).await;
        
        if move_actuators(host_handle, None, Some(2000)).await.called {
            return SoftStopped { called: true };
        }

        tokio::select! {
            _ = tokio::time::sleep(Duration::from_millis(3000)) => {}
            _ = async {
                loop {
                    host_handle.write_to_host(FromAI::SetSteering(Steering::new(-0.5, -0.5, Steering::DEFAULT_WEIGHT)));
                    if let Some(FromHost::FromLunabase { msg: FromLunabase::SoftStop }) = host_handle.try_read_from_host() {
                        return SoftStopped { called: true };
                    }
                    tokio::time::sleep(Duration::from_millis(80)).await;
                }
            } => {}
        }
        host_handle.write_to_host(FromAI::SetSteering(Steering::default()));
    }
    
}