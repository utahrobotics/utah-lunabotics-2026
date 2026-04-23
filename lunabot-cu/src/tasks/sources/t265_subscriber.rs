#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crossbeam::channel::Receiver;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use rerun::Vector3D;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::{collections::HashMap, sync::Arc};

use cu_bincode::{Decode, Encode};
use cu_sensor_payloads::CuImage;
use cu_spatial_payloads::EncodableIsometry;
use cu29::{
    cutask::{CuSrcTask, Freezable},
    prelude::*,
};

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use nalgebra::{Isometry3, UnitQuaternion, Vector3};
use serde::Deserialize;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::ops::DerefMut;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use t265_rs::{Pose, T265Manager, VideoFrame};

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crate::ROBOT_STATE;

#[cfg(all(any(feature = "resim", feature = "sim")))]
pub struct T265Subscriber {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
pub struct T265Subscriber {
    last_process_call: std::time::Instant,
    last_seen: u64,
    last_seen_img: u64,

    velocity_variance: f64,
    pose_variance: f64,
    angular_velocity_variance: f64,
    _manager: T265Manager,
    image_rx: Receiver<VideoFrame>,
    pose_rx: Receiver<Pose>,
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,

    /// serial, warmupstate
    warmup_states: HashMap<String, WarmupState>,

    pub left_serial: String,
    pub right_serial: String,
    pub rear_serial: String,
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
#[derive(Clone)]
pub struct WarmupState {
    done: bool,
    current_pose_count: usize,
    warmup_pose_count: usize,
    twist_correction: Option<UnitQuaternion<f64>>,
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl Default for WarmupState {
    fn default() -> Self {
        Self {
            done: false,
            current_pose_count: 0,
            warmup_pose_count: 100,
            twist_correction: Default::default(),
        }
    }
}

#[derive(Encode, Decode, Clone, Serialize, Debug, Deserialize)]
pub struct T265Msg {
    /// the pose of the robot base
    pub pose: EncodableIsometry,
    pub pose_variance: f64,
    pub velocity_variance: f64,
    pub angular_velocity_variance: f64,
    /// always set to the serial number
    pub node_name: String,
    pub imu_msg: T265IMUMsg,
}

#[derive(Encode, Decode, Clone, Serialize, Debug, Deserialize)]
pub struct T265IMUMsg {
    pub accel: [f32; 3],
    pub angular_accel: [f32; 3],
    pub velocity: [f32; 3],
    pub angular_velocity: [f32; 3],
}

impl Default for T265IMUMsg {
    fn default() -> Self {
        Self {
            accel: [0.0, 0.0, 0.0],
            angular_accel: [0.0, 0.0, 0.0],
            velocity: [0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
        }
    }
}

impl Default for T265Msg {
    fn default() -> Self {
        Self {
            pose: EncodableIsometry::default(),
            velocity_variance: 1.0,
            pose_variance: 0.5,
            angular_velocity_variance: 1.0,
            node_name: "t265".to_string(),
            imu_msg: Default::default(),
        }
    }
}

impl Freezable for T265Subscriber {}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl CuSrcTask for T265Subscriber {
    // combined t265 messages, left hand side t265, right hand side t265, rear t265
    type Output<'m> = output_msg!(
        T265Msg,
        CuImage<Vec<u8>>,
        CuImage<Vec<u8>>,
        CuImage<Vec<u8>>
    );
    type Resources<'r> = ();

    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        let velocity_variance = config
            .and_then(|c| {
                c.get::<f64>("t265_velocity_variance")
                    .expect("failed to deserialize")
            })
            .unwrap_or(1.0);

        let pose_variance = config
            .and_then(|c| {
                c.get::<f64>("pose_variance")
                    .expect("failed to deserialize")
            })
            .unwrap_or(1.0);

        let angular_velocity_variance = config
            .and_then(|c| {
                c.get::<f64>("t265_angular_velocity_variance")
                    .expect("failed to deserialize")
            })
            .unwrap_or(1.0);

        let left_serial = config
            .and_then(|c| {
                c.get::<String>("left_serial")
                    .expect("failed to deserialize")
            })
            .expect("Please provide left t261 serial");
        let right_serial = config
            .and_then(|c| {
                c.get::<String>("right_serial")
                    .expect("failed to deserialize")
            })
            .expect("Please provide right t261 serial");

        let rear_serial = config
            .and_then(|c| {
                c.get::<String>("rear_serial")
                    .expect("failed to deserialize")
            })
            .expect("Please provide rear t261 serial");

        let mut manager = T265Manager::new().map_err(|e| CuError::from(e.to_string()))?;
        manager
            .discover_devices_with_options(true)
            .map_err(|e| CuError::from(e.to_string()))?;
        manager
            .enable_all_video_streams()
            .map_err(|e| CuError::from(e.to_string()))?;

        let pose_rx = manager
            .start_all_pose_streams()
            .map_err(|e| CuError::from(e.to_string()))?;
        let image_rx = manager
            .start_all_video_streams()
            .map_err(|e| CuError::from(e.to_string()))?;
        const WIDTH: usize = 848;
        const HEIGHT: usize = 800;
        let pool = CuHostMemoryPool::new("t265_imgs", 12, || vec![0u8; (WIDTH * HEIGHT) as usize])?;

        Ok(Self {
            last_process_call: std::time::Instant::now(),
            last_seen_img: 0,
            last_seen: 0,
            velocity_variance,
            angular_velocity_variance,
            _manager: manager,
            pose_rx,
            image_rx,
            pool,
            pose_variance,
            warmup_states: [
                (left_serial.clone(), WarmupState::default()),
                (right_serial.clone(), WarmupState::default()),
                (rear_serial.clone(), WarmupState::default()),
            ]
            .into(),
            left_serial,
            right_serial,
            rear_serial,
        })
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        if self.last_process_call.elapsed().as_millis() > 5 {
            eprintln!(
                "[WARNING] process not called for 5 miliseconds: took {}s",
                self.last_process_call.elapsed().as_secs_f64()
            );
        }
        self.last_process_call = std::time::Instant::now();
        new_msg.0.clear_payload();
        new_msg.1.clear_payload();
        new_msg.2.clear_payload();

        'image_rx: {
            if self.image_rx.len() > 5 {
                eprintln!("WARNING: t265 image backpressure: {}", self.image_rx.len());
                while self.image_rx.len() > 1 {
                    let _ = self.image_rx.try_recv();
                }
            }
            if let Ok(mut image) = self.image_rx.try_recv() {
                if image.sensor_index == 0 {
                    break 'image_rx;
                }
                self.last_seen_img = clock.now().as_millis();
                use cu_sensor_payloads::CuImageBufferFormat;
                let Some(handle) = self.pool.acquire() else {
                    return Err(CuError::from("No handle available for t265"));
                };
                if ROBOT_STATE
                    .get()
                    .unwrap()
                    .kinematic_root
                    .get_node_with_name(&image.device_id)
                    .is_none()
                {
                    break 'image_rx;
                }
                handle.with_inner_mut(|inner| {
                    match inner {
                        CuHandleInner::Pooled(reusable) => {
                            let dst: &mut Vec<u8> = reusable.deref_mut();
                            if dst.len() != image.data.len() {
                                return Err(CuError::from("length mismatch"));
                            }
                            std::mem::swap(dst, &mut image.data);
                        }
                        CuHandleInner::Detached(v) => {
                            if v.len() != image.data.len() {
                                return Err(CuError::from("length mismatch"));
                            }
                            std::mem::swap(v, &mut image.data);
                        }
                    }
                    Ok(())
                })?;

                let cu_image_handle = CuImage::new(
                    CuImageBufferFormat {
                        width: image.width as u32,
                        height: image.height as u32,
                        stride: image.stride as u32,
                        pixel_format: *b"GRAY",
                    },
                    handle,
                );

                if image.device_id == self.left_serial {
                    new_msg.1.set_payload(cu_image_handle);
                } else if image.device_id == self.right_serial {
                    new_msg.2.set_payload(cu_image_handle);
                } else if image.device_id == self.rear_serial {
                    new_msg.3.set_payload(cu_image_handle);
                } else {
                    eprintln!("[T265 SUBSCRIBER] Unknown serial on image");
                }
            }
        }

        if self.pose_rx.len() > 5 {
            eprintln!("WARNING: t265 pose backpressure: {}", self.pose_rx.len());
            // drain off backpressure
            while self.pose_rx.len() > 1 {
                let _ = self.pose_rx.try_recv();
            }
        }

        if let Ok(Pose {
            translation,
            rotation,
            velocity,
            angular_velocity,
            acceleration,
            angular_acceleration,
            timestamp_ns: _,
            tracker_confidence: _,
            mapper_confidence: _,
            tracker_state: _,
            device_id,
        }) = self.pose_rx.try_recv()
        {
            let Some(warmup_state) = self.warmup_states.get_mut(&device_id) else {
                return Err(CuError::from(format!("Unknown serial {device_id}")));
            };
            if !warmup_state.done
                && warmup_state.current_pose_count < warmup_state.warmup_pose_count
            {
                warmup_state.current_pose_count += 1;
                return Err(CuError::from(format!("{device_id} warming up")));
            } else {
                // its just nice to stop counting, not that it actually changes performance in any way
                warmup_state.done = true;
            }

            use nalgebra::Quaternion;

            use crate::ROBOT_STATE;
            // Coordinate frame transform from T265 to robot (same as translation: [-z, -x, y])
            // https://stackoverflow.com/questions/18818102/convert-quaternion-representing-rotation-from-one-coordinate-system-to-another
            let coord_transform =
                UnitQuaternion::from_quaternion(Quaternion::new(0.5, 0.5, -0.5, -0.5));
            let q_t265 = UnitQuaternion::from_quaternion(Quaternion::new(
                rotation[3],
                rotation[0],
                rotation[1],
                rotation[2],
            ));

            let q_robot = coord_transform * q_t265 * coord_transform.inverse();
            let pose = Isometry3::from_parts(
                Vector3::new(-translation[2], -translation[0], translation[1]).into(),
                q_robot,
            );

            // transforming accel vectors to  z up y left x forward
            //see misc/t265_translations/t265_transformations.py

            use nalgebra::Matrix3;
            let x_angle = std::f64::consts::PI;
            let z_angle = std::f64::consts::FRAC_PI_2;

            let accel_rotation_matrix_x = Matrix3::new(
                1.0,
                0.0,
                0.0,
                0.0,
                x_angle.cos(),
                -x_angle.sin(),
                0.0,
                x_angle.sin(),
                x_angle.cos(),
            );

            let accel_rotation_matrix_z = Matrix3::new(
                z_angle.cos(),
                -z_angle.sin(),
                0.0,
                z_angle.sin(),
                z_angle.cos(),
                0.0,
                0.0,
                0.0,
                1.0,
            );

            let accel_rotation_matrix = accel_rotation_matrix_z * accel_rotation_matrix_x;

            let accel_vector: Vector3<f64> = Vector3::from_row_slice(&acceleration);
            let angular_accel_vector: Vector3<f64> = Vector3::from_row_slice(&angular_acceleration);
            let velo_vector: Vector3<f64> = Vector3::from_row_slice(&velocity);
            let angular_velo_vector: Vector3<f64> = Vector3::from_row_slice(&angular_velocity);

            let transformed_accel = accel_rotation_matrix * accel_vector;
            let transformed_ang_accel = accel_rotation_matrix * angular_accel_vector;
            let transformed_velo = accel_rotation_matrix * velo_vector;
            let transformed_ang_velo = accel_rotation_matrix * angular_velo_vector;

            let accel: [f32; 3] = [
                transformed_accel.x as f32,
                transformed_accel.y as f32,
                transformed_accel.z as f32,
            ];

            let angular_acceleration: [f32; 3] = [
                transformed_ang_accel.x as f32,
                transformed_ang_accel.y as f32,
                transformed_ang_accel.z as f32,
            ];

            let velocity: [f32; 3] = [
                transformed_velo.x as f32,
                transformed_velo.y as f32,
                transformed_velo.z as f32,
            ];

            let angular_velocity: [f32; 3] = [
                transformed_ang_velo.x as f32,
                transformed_ang_velo.y as f32,
                transformed_ang_velo.z as f32,
            ];

            let Some(kinematic_node) = ROBOT_STATE
                .get()
                .expect("robot state not initialized")
                .kinematic_root
                .get_node_with_name(&device_id)
            else {
                return Err(CuError::new_with_cause(
                    "received pose from unknown device",
                    std::io::Error::other("received pose from unknown device"),
                ));
            };

            let twist_correction = warmup_state.twist_correction.unwrap_or_else(|| {
                use nalgebra::UnitVector3;

                use crate::utils::swing_twist_decomposition;

                let robot_base = pose.cast() * kinematic_node.get_isometry_from_base().inverse();
                let up: Vector3<f64> = Vector3::z();
                let (_, twist) = swing_twist_decomposition(
                    &robot_base.rotation,
                    &UnitVector3::new_normalize(up),
                );
                warmup_state.twist_correction = Some(twist.inverse());
                twist.inverse()
            });

            let base_twist_corrected_pose =
                twist_correction * pose.cast() * kinematic_node.get_isometry_from_base().inverse();
            let msg = T265Msg {
                pose: EncodableIsometry::from_na(&base_twist_corrected_pose),
                pose_variance: self.pose_variance,
                velocity_variance: self.velocity_variance,
                angular_velocity_variance: self.angular_velocity_variance,
                node_name: device_id,
                imu_msg: T265IMUMsg {
                    accel: accel,
                    angular_accel: angular_acceleration,
                    velocity,
                    angular_velocity,
                },
            };
            new_msg.0.set_payload(msg);
            self.last_seen = clock.now().as_millis();
        }

        let mut err_msg = None;
        if clock.now().as_millis() - self.last_seen > 500 {
            err_msg = Some("no poses seen in 500 ms".to_string());
        }

        if clock.now().as_millis() - self.last_seen_img > 500 {
            *err_msg.get_or_insert(String::new()) += " No images seen in 500 ms";
        }

        if let Some(ref err_msg) = err_msg {
            return Err(CuError::from(err_msg.as_str()));
        }

        Ok(())
    }
}

#[cfg(any(feature = "resim", feature = "sim"))]
impl CuSrcTask for T265Subscriber {
    type Output<'m> = output_msg!(
        T265Msg,
        CuImage<Vec<u8>>,
        CuImage<Vec<u8>>,
        CuImage<Vec<u8>>
    );
    type Resources<'r> = ();
    fn new(
        _config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self {})
    }
    fn process<'o>(
        &mut self,
        _clock: &cu29::prelude::RobotClock,
        _new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        Ok(())
    }
}
