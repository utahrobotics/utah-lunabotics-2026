use std::time::Duration;

use common::{FromAI, FromHost, FromLunabase, LunabotStage, Steering};
use embedded_common::{Actuator, ActuatorCommand};
use nalgebra::Vector2;
use tokio::time::Instant;

use crate::context::HostHandle;

mod traverse_simple;
mod dig_dump;
mod dig_dump_simple;

struct SoftStopped {
    pub called: bool
}

pub async fn teleop(host_handle: &mut HostHandle) {
    host_handle.write_to_host(FromAI::SetStage(LunabotStage::TeleOp));
    let mut last_lift = FromLunabase::set_lift_actuator(0.0);
    let mut last_bucket = FromLunabase::set_bucket_actuator(0.0);
    let mut last_steering = Steering::default();
    let mut drive_instant = Instant::now();

    loop {
        let msg;
        
        loop {
            tokio::select! {
                x = host_handle.read_from_host() => {
                    msg = x;
                    break;
                }
                _ = tokio::time::sleep_until(drive_instant) => {
                    drive_instant = Instant::now() + Duration::from_millis(80);

                    let [msg1, msg2] = last_lift.get_lift_actuator_commands().unwrap();
                    host_handle.write_to_host(FromAI::SetActuators(msg1));
                    host_handle.write_to_host(FromAI::SetActuators(msg2));

                    let [msg1, msg2] = last_bucket.get_bucket_actuator_commands().unwrap();
                    host_handle.write_to_host(FromAI::SetActuators(msg1));
                    host_handle.write_to_host(FromAI::SetActuators(msg2));

                    host_handle.write_to_host(FromAI::SetSteering(last_steering));
                }
            }
        }

        let FromHost::FromLunabase { msg } = msg else {
            continue;
        };
        match msg {
            FromLunabase::LiftActuators(_) => {
                last_lift = msg;
            }
            FromLunabase::BucketActuators(_) => {
                last_bucket = msg;
            }
            FromLunabase::Steering(steering) => {
                last_steering = steering;
            }
            FromLunabase::Navigate((x, y)) => {
                eprintln!("Navigational autonomy not implemented");
                host_handle.write_to_host(FromAI::SetStage(LunabotStage::TeleOp));
            }
            FromLunabase::DigDump(_) => {
                if dig_dump_simple::dig_dump_simple(host_handle).await.called {
                    break;
                }
                host_handle.write_to_host(FromAI::SetStage(LunabotStage::TeleOp));
            }
            FromLunabase::SoftStop => break,
            FromLunabase::StartPercuss => host_handle.write_to_host(FromAI::StartPercuss),
            FromLunabase::StopPercuss => host_handle.write_to_host(FromAI::StopPercuss),
            _ => {}
        }
    }
}

const ACTUATOR_COMPLETION_THRESHOLD: u16 = 100;

async fn move_actuators(host_handle: &mut HostHandle, mut target_lift: Option<u16>, mut target_bucket: Option<u16>) -> SoftStopped {
    loop {
        match host_handle.read_from_host().await {
            FromHost::FromLunabase { msg } => match msg {
                FromLunabase::SoftStop => return SoftStopped { called: true },
                _ => {}
            },
            FromHost::ActuatorReadings { lift, bucket } => {
                if let Some(target_lift_unwrapped) = target_lift {
                    let mut lift_diff = target_lift_unwrapped.checked_signed_diff(lift).unwrap().clamp(-1000, 1000) as f64;
                    println!("Lift diff: {lift_diff}");

                    if lift_diff.abs() > ACTUATOR_COMPLETION_THRESHOLD as f64 {
                        if lift_diff.abs() < 200.0 {
                            lift_diff = 200.0 * lift_diff.signum();
                        }
                        host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(-lift_diff / 1000.0, Actuator::Lift)));
                    } else {
                        target_lift = None;
                    }
                }
                if let Some(target_bucket_unwrapped) = target_bucket {
                    let mut bucket_diff = target_bucket_unwrapped.checked_signed_diff(bucket).unwrap().clamp(-1000, 1000) as f64;
                    println!("Bucket diff: {bucket_diff}");
                    
                    if bucket_diff.abs() > ACTUATOR_COMPLETION_THRESHOLD as f64 {
                        if bucket_diff.abs() < 200.0 {
                            bucket_diff = 200.0 * bucket_diff.signum();
                        }
                        host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(bucket_diff / 1000.0, Actuator::Bucket)));
                    } else {
                        target_bucket = None;
                    }
                }
                if target_bucket.is_none() && target_lift.is_none() {
                    println!("Move actuators done");
                        host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(0.0, Actuator::Lift)));
                        host_handle.write_to_host(FromAI::SetActuators(ActuatorCommand::set_speed(0.0, Actuator::Bucket)));
                    return SoftStopped { called: false };
                }
            }
            _ => {}
        }
    }
}