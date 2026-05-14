use cu29::{
    CuResult,
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{Freezable},
    prelude::*,
};
use bincode::{Encode, Decode};

use common::Steering;
use serde::Deserialize;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use vesc_translator::GetValuesResponse;

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crate::motors::{MotorRef, VescIDs, VescPair, enumerate_motors};

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]

pub struct MotorController {
    motor_ref: &'static MotorRef,
    prev_speed_multi: f32, /*  */
    last_seen: Option<Instant>,
}

#[derive(Encode, Decode, Serialize, Deserialize, Clone, Copy, Debug, Default)]
pub struct EncodableGetValuesResponse {
    pub temp_mos: f32,
    pub temp_motor: f32,
    pub motor_current: f32,
    pub input_current: f32,
    pub avg_id: f32,
    pub avg_iq: f32,
    pub duty_cycle_now: f32,
    pub rpm: f32,
    pub v_in: f32,
    pub amp_hours: f32,
    pub amp_hours_charged: f32,
    pub watt_hours: f32,
    pub watt_hours_charged: f32,
    pub tachometer: i32,
    pub tachometer_abs: i32,
    pub fault_code: u8,
    pub pid_pos_now: f32,
    pub vesc_id: u8,
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl From<&GetValuesResponse> for EncodableGetValuesResponse {
    fn from(value: &GetValuesResponse) -> Self {
        Self {
            temp_mos: value.temp_mos,
            temp_motor: value.temp_motor,
            motor_current: value.motor_current,
            input_current: value.input_current,
            avg_id: value.avg_id,
            avg_iq: value.avg_iq,
            duty_cycle_now: value.duty_cycle_now,
            rpm: value.rpm,
            v_in: value.v_in,
            amp_hours: value.amp_hours,
            amp_hours_charged: value.amp_hours_charged,
            watt_hours: value.watt_hours,
            watt_hours_charged: value.watt_hours_charged,
            tachometer: value.tachometer,
            tachometer_abs: value.tachometer_abs,
            fault_code: value.fault_code,
            pid_pos_now: value.pid_pos_now,
            vesc_id: value.vesc_id,
        }
    }
}

impl Freezable for MotorController {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuTask for MotorController {
    // steering, actuators (just ignore the actuators here for now)
    type Input<'m> = input_msg!(Steering);
    type Output<'m> = output_msg!(EncodableGetValuesResponse);


    type Resources<'r> = ();

    fn new(config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        let motor_ref;
        let prev_speed_multi: f32;

        if let Some(config) = config {
            let mut vesc_ids = VescIDs::default();
            let speed_multiplier = config
                .get::<f64>("speed_multiplier")
                .expect("failed to deserialize")
                .unwrap_or(2000.) as f32;
            let invert_left = config
                .get::<bool>("invert_left")
                .expect("failed to deserialize invert_left")
                .unwrap_or(false);
            let invert_right = config
                .get::<bool>("invert_right")
                .expect("failed to deserialize invert_right")
                .unwrap_or(false);
            let vesc_pairs = config
                .get_value::<Vec<VescPair>>("vesc_pairs")
                .expect("failed to deserialize vesc pairs")
                .expect("vesc pairs not found");
            prev_speed_multi = speed_multiplier;
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
            motor_ref = enumerate_motors(vesc_ids, speed_multiplier, invert_left, invert_right).expect("failed to call enumerate_motors");
        } else {
            return Err(CuError::new_with_cause(
                "no vesc pairs or speed multiplier set in config",
                std::io::Error::other("no vesc pairs or speed multiplier set in config"),
            ));
        }

        Ok(Self {
            motor_ref,
            prev_speed_multi,
            last_seen: None,
        })
    }
    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        self.motor_ref.set_speed_multiplier(self.prev_speed_multi);
        Ok(())
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, input: &Self::Input<'_>, output: &mut Self::Output<'_>) -> CuResult<()> {
        if let Some(steering) = input.payload() {
            let new_weight: f32 = steering.get_weight() as f32;
            if (new_weight - self.prev_speed_multi).abs() > 0.0001 {
                self.prev_speed_multi = new_weight;
                self.motor_ref.set_speed_multiplier(new_weight);
            }
            let (left, right) = steering.get_left_and_right();
            self.motor_ref.set_speed(left as f32, right as f32);
        }
    

        if let Some(telemetry) = self.motor_ref.get_latest_telemetry() {
            use crate::rerun_viz::RECORDER;

            if let Some(rec) = RECORDER.get() {
                for item in telemetry.iter() {
                    let id = item.vesc_id;
                    use rerun::TextLog;
                    let _ = rec
                        .recorder
                        .log(format!("vescs/{id}"), &TextLog::new(format!("{item:?}")));
                    output.set_payload(item.into());
                }
            } else {
                // println!("{telemetry:?}");
            }
            self.last_seen = Some(Instant::now());
        }

        if let Some(errors) = self.motor_ref.get_latest_errors() {
            return Err(CuError::from(errors.join(" | ")));
        }

        if let Some(last_seen) = self.last_seen && (Instant::now().as_nanos() - last_seen.as_nanos()) > 900_000_000 {
            return Err(CuError::from("No telemetry seen in 900ms"));
        }

        Ok(())
    }    
}

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]
pub struct MotorController;

#[cfg(any(not(target_os = "linux"), feature = "resim", feature = "sim"))]

impl CuTask for MotorController {
    type Input<'m> = input_msg!(Steering);
    type Output<'m> = output_msg!(EncodableGetValuesResponse);

    type Resources<'r> = ();
    fn new(_config: Option<&ComponentConfig>, _resources: Self::Resources<'_>) -> CuResult<Self> {
        Ok(Self {})
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn process(&mut self, _clock: &RobotClock, _input: &Self::Input<'_>, _output: &mut Self::Output<'_>) -> CuResult<()> {
        Ok(())
    }


    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}
