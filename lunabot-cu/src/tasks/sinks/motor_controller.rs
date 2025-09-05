use cu29::{
    CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSinkTask, Freezable},
    prelude::*,
};

use common::Steering;

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crate::motors::{MotorRef, VescIDs, VescPair, enumerate_motors};

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]

pub struct MotorController {
    motor_ref: &'static MotorRef,
}

impl Freezable for MotorController {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuSinkTask for MotorController {
    // steering, actuators (just ignore the actuators here for now)
    type Input<'m> = input_msg!((Option<Steering>, Option<[u8; 5]>));

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let motor_ref;
        if let Some(config) = config
            && let Some(vesc_pairs) = config.get::<Vec<VescPair>>("vesc_pairs")
        {
            let mut vesc_ids = VescIDs::default();
            let speed_multiplier = config.get::<f64>("speed_multiplier").unwrap_or(2000.) as f32;
            for VescPair {
                id1,
                id2,
                mask1,
                mask2,
                command_both,
            } in vesc_pairs
            {
                if vesc_ids.add_dual_vesc(id1, id2, mask1, mask2, command_both) {
                    return Err(CuError::new_with_cause(
                        "motors have already been added",
                        std::io::Error::new(
                            std::io::ErrorKind::AlreadyExists,
                            format!("Motors {id1} or {id2} have already been added"),
                        ),
                    ));
                }
            }
            motor_ref = enumerate_motors(vesc_ids, speed_multiplier);
        } else {
            return Err(CuError::new_with_cause(
                "no vesc pairs or speed multiplier set in config",
                std::io::Error::other("no vesc pairs or speed multiplier set in config"),
            ));
        }

        Ok(Self { motor_ref })
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>) -> CuResult<()> {
        if let Some(payload) = input.payload() {
            if let Some(steering) = &payload.0 {
                let (left, right) = steering.get_left_and_right();
                self.motor_ref.set_speed(left as f32, right as f32);
            }
        }
        Ok(())
    }
}

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
pub struct MotorController;

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
impl CuSinkTask for MotorController {
    type Input<'m> = input_msg!((Option<Steering>, Option<[u8; 5]>));
    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self {})
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
