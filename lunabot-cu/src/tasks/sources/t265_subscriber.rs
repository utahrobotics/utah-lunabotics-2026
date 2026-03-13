#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::sync::{Arc, mpsc::Receiver};

use cu_bincode::{Decode, Encode};
use cu_sensor_payloads::CuImage;
use cu_spatial_payloads::EncodableIsometry;
use cu29::{
    cutask::{CuSrcTask, Freezable},
    prelude::*,
};

use nalgebra::{Isometry3, UnitQuaternion, Vector3};
use serde::Deserialize;
use simple_motion::StaticNode;
use std::ops::DerefMut;
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use t265_rs::{ImuFrame, Pose, T265Manager, VideoFrame};

#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use crate::{ROBOT_STATE, utils::swing_twist_decomposition};

pub struct T265Subscriber {
    last_seen: u64,

    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    warmup_ms: usize,

    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    first_pose: Option<std::time::Instant>,


    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    twist_correction: Option<UnitQuaternion<f64>>,

    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    velocity_variance: f64,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    pose_variance: f64,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    angular_velocity_variance: f64,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    manager: T265Manager,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    imu_rx: Receiver<ImuFrame>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    image_rx: Receiver<VideoFrame>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    pose_rx: Receiver<Pose>,
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    pool: Arc<CuHostMemoryPool<Vec<u8>>>,
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
    type Output<'m> = output_msg!(T265Msg, CuImage<Vec<u8>>);
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

        let mut manager = T265Manager::new().map_err(|e| CuError::from(e.to_string()))?;
        manager
            .discover_devices_with_options(true)
            .map_err(|e| CuError::from(e.to_string()))?;
        manager
            .enable_all_video_streams()
            .map_err(|e| CuError::from(e.to_string()))?;

        let imu_rx = manager
            .start_all_imu_streams()
            .map_err(|e| CuError::from(e.to_string()))?;
        let pose_rx = manager
            .start_all_pose_streams()
            .map_err(|e| CuError::from(e.to_string()))?;
        let image_rx = manager
            .start_all_video_streams()
            .map_err(|e| CuError::from(e.to_string()))?;
        const WIDTH: usize = 848;
        const HEIGHT: usize = 800;
        let pool = CuHostMemoryPool::new("t265_imgs", 4, || vec![0u8; (WIDTH * HEIGHT) as usize])?;

        Ok(Self {
            warmup_ms: 500,
            first_pose: None,
            last_seen: 0,
            velocity_variance,
            angular_velocity_variance,
            manager,
            imu_rx,
            pose_rx,
            image_rx,
            pool,
            pose_variance,
            twist_correction: None,
        })
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        new_msg.0.clear_payload();
        new_msg.1.clear_payload();

        'image_rx: {
            if let Ok(image) = self.image_rx.try_recv() {
                use cu_sensor_payloads::CuImageBufferFormat;

                let Some(handle) = self.pool.acquire() else {
                    return Err(CuError::from("No handle available for t265"));
                };
                // we only want images from the righthand fisheye lense
                if image.sensor_index == 0 {
                    break 'image_rx;
                }
                handle.with_inner_mut(|inner| {
                    let dst = inner.deref_mut();
                    if dst.len() != image.data.len() {
                        return Err(CuError::from("handle data len doesnt match image data len"));
                    }
                    dst.copy_from_slice(&image.data);
                    Ok(())
                })?;

                let image = CuImage::new(
                    CuImageBufferFormat {
                        width: image.width as u32,
                        height: image.height as u32,
                        stride: image.stride as u32,
                        pixel_format: *b"GRAY",
                    },
                    handle,
                );
                new_msg.1.set_payload(image);
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
            tracker_confidence,
            mapper_confidence: _,
            tracker_state: _,
            device_id,
        }) = self.pose_rx.try_recv()
        {
            if self.first_pose.get_or_insert(std::time::Instant::now()).elapsed().as_millis() < self.warmup_ms as u128 {
                return Err(CuError::new_with_cause(
                    "Warming up...",
                    std::io::Error::other("Warming up..."),
                ));
            }
            
            use nalgebra::Quaternion;
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

            let twist_correction = self.twist_correction.unwrap_or_else(|| {
                use nalgebra::UnitVector3;

                let robot_base = pose.cast() * kinematic_node.get_isometry_from_base().inverse();
                let up:Vector3<f64> = Vector3::z();
                let (_, twist) = swing_twist_decomposition(&robot_base.rotation, &UnitVector3::new_normalize(up));
                self.twist_correction = Some(twist.inverse());
                twist.inverse()
            });

            let base_twist_corrected_pose = twist_correction * pose.cast() * kinematic_node.get_isometry_from_base().inverse();



            let msg = T265Msg {
                pose: EncodableIsometry::from_na(&base_twist_corrected_pose),
                pose_variance: self.pose_variance,
                velocity_variance: self.velocity_variance,
                angular_velocity_variance: self.angular_velocity_variance,
                node_name: device_id,
                imu_msg: T265IMUMsg {
                    accel: acceleration,
                    angular_accel: angular_acceleration,
                    velocity,
                    angular_velocity,
                },
            };
            new_msg.0.set_payload(msg);
        }

        if clock.now().as_nanos() - self.last_seen > 500_000_000 {
            return Err(CuError::new_with_cause(
                "no pose frames seen in 500 ms",
                std::io::Error::other("no pose frames seen in 500 ms"),
            ));
        }

        Ok(())
    }
}

#[cfg(any(feature = "resim", feature = "sim"))]
impl CuSrcTask for T265Subscriber {
    type Output<'m> = output_msg!(T265Msg, CuImage<Vec<u8>>);
    type Resources<'r> = ();
    fn new(
        config: Option<&cu29::prelude::ComponentConfig>,
        _resources: Self::Resources<'_>,
    ) -> cu29::CuResult<Self>
    where
        Self: Sized,
    {
        Ok(Self { last_seen: 0 })
    }
    fn process<'o>(
        &mut self,
        _clock: &cu29::prelude::RobotClock,
        _new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        Ok(())
    }
}
