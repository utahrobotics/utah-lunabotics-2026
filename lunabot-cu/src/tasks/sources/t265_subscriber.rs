#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crossbeam::channel::Receiver;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::{collections::HashMap,sync::Arc};

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
use crate::{ROBOT_STATE, kalman_filtering::quaternion_error};

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

    synchronizer: Synchronizer,
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
#[derive(Clone)]
pub struct WarmupState {
    done: bool,
    current_pose_count: usize,
    warmup_pose_count: usize,
    // the t265's pose readings have an arbitrary origin.
    world_to_odom_offset: Option<Isometry3<f64>>, 
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl Default for WarmupState {
    fn default() -> Self {
        Self {
            done: false,
            current_pose_count: 0,
            warmup_pose_count: 100,
            world_to_odom_offset: None,
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
            synchronizer: Synchronizer::new()
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

            let isometry_from_base = kinematic_node.get_isometry_from_base();
            let current_raw_pose: Isometry3<f64> = pose.cast();

            // Calculate or retrieve the static offset to align this sensor's odometry 
            // origin to the robot's world origin established directly after warmup.
            let world_to_odom = warmup_state.world_to_odom_offset.unwrap_or_else(|| {
                let offset = isometry_from_base * current_raw_pose.inverse();
                warmup_state.world_to_odom_offset = Some(offset);
                offset
            });

            // World -> Odom -> Sensor -> Base
            let base_pose_in_world = world_to_odom * current_raw_pose * isometry_from_base.inverse();

            let msg = T265Msg {
                pose: EncodableIsometry::from_na(&base_pose_in_world),
                pose_variance: self.pose_variance,
                velocity_variance: self.velocity_variance,
                angular_velocity_variance: self.angular_velocity_variance,
                imu_msg: T265IMUMsg {
                    accel: acceleration,
                    angular_accel: angular_acceleration,
                    velocity,
                    angular_velocity,
                },
            };
            self.synchronizer.push_pose(device_id, msg);
            if let Some(latest) = self.synchronizer.get_fused(7) {
                new_msg.0.set_payload(latest);
            }
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

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
pub struct Synchronizer {
    pub buffer: HashMap<String, (T265Msg, Instant)>,
}

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
impl Synchronizer {
    pub fn new() -> Self {
        Self {
            buffer: HashMap::new(),
        }
    }

    pub fn push_pose(&mut self, key: String, pose: T265Msg) {
        self.buffer.insert(key, (pose, Instant::now()));
    }

    pub fn get_fused(&mut self, max_age_ms: u64) -> Option<T265Msg> {
        self.buffer
            .retain(|_, (_, t)| Instant::now().as_nanos() - t.as_nanos() < (max_age_ms * 1000000));

        let fresh: Vec<&T265Msg> = self.buffer.values().map(|(msg, _)| msg).collect();

        if fresh.is_empty() {
            return None;
        }
        if fresh.len() == 1 {
            return Some(fresh[0].clone());
        }

        let weights: Vec<f64> = fresh.iter().map(|m| 1.0 / m.pose_variance).collect();
        let weight_sum: f64 = weights.iter().sum();

        let reference_quat = fresh
            .iter()
            .zip(&weights)
            .max_by(|(_, wa), (_, wb)| wa.partial_cmp(wb).unwrap())
            .and_then(|(msg, _)| msg.pose.to_na())
            .map(|iso| iso.rotation)?;

        let mut fused_translation = Vector3::zeros();
        let mut fused_rotation_error = Vector3::zeros();
        let mut fused_imu = T265IMUMsg::default();

        for (msg, w) in fresh.iter().zip(&weights) {
            let normalized_w = w / weight_sum;
            let Some(iso) = msg.pose.to_na() else { continue };

            fused_translation += iso.translation.vector * normalized_w;

            let err = quaternion_error(&reference_quat, &iso.rotation);
            fused_rotation_error += err * normalized_w;

            let imu = &msg.imu_msg;
            for i in 0..3 {
                fused_imu.accel[i] += (imu.accel[i] as f64 * normalized_w) as f32;
                fused_imu.velocity[i] += (imu.velocity[i] as f64 * normalized_w) as f32;
                fused_imu.angular_velocity[i] +=
                    (imu.angular_velocity[i] as f64 * normalized_w) as f32;
                fused_imu.angular_accel[i] +=
                    (imu.angular_accel[i] as f64 * normalized_w) as f32;
            }
        }

        let fused_rotation =
            UnitQuaternion::from_scaled_axis(fused_rotation_error) * reference_quat;
        let fused_pose = Isometry3::from_parts(fused_translation.into(), fused_rotation);

        let fused_variance = 1.0 / weight_sum;

        Some(T265Msg {
            pose: EncodableIsometry::from_na(&fused_pose),
            pose_variance: fused_variance,
            velocity_variance: 1.0 / fresh.iter().zip(&weights)
                .map(|(m, w)| w / m.velocity_variance)
                .sum::<f64>(),
            angular_velocity_variance: 1.0 / fresh.iter().zip(&weights)
                .map(|(m, w)| w / m.angular_velocity_variance)
                .sum::<f64>(),
            imu_msg: fused_imu,
        })
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