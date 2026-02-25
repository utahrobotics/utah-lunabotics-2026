#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
use std::sync::{Arc, mpsc::Receiver};

use cu_bincode::{Decode, Encode};
#[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
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

pub struct T265Subscriber {
    last_seen: u64,
    /// Initial yaw offset captured on first pose to align T265's arbitrary tracking frame with world
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    initial_yaw_offset: Option<f32>,

    /// we only use the deltas between poses in the localizer
    #[cfg(all(target_os = "linux", not(any(feature = "resim", feature = "sim"))))]
    velocity_variance: f64,
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
    pub pose: EncodableIsometry,
    pub velocity_variance: f64,
    pub angular_velocity_variance: f64,
    pub node_name: String,
    pub imu_msg: T265IMUMsg,
}

#[derive(Encode, Decode, Clone, Serialize, Debug, Deserialize)]
pub struct T265IMUMsg {
    pub accel: [f32; 3],
    pub angular_accel: [f32; 3],
}

impl Default for T265IMUMsg {
    fn default() -> Self {
        Self {
            accel: [0.0, 0.0, 0.0],
            angular_accel: [0.0, 0.0, 0.0],
        }
    }
}

impl Default for T265Msg {
    fn default() -> Self {
        Self {
            pose: EncodableIsometry::default(),
            velocity_variance: 1.0,
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
        let serial_num = config
            .and_then(|c| {
                c.get::<String>("serial_num")
                    .expect("failed to deserialize")
            })
            .unwrap_or_else(|| "realsense/t265".to_string());

        let velocity_variance = config
            .and_then(|c| {
                c.get::<f64>("t265_velocity_variance")
                    .expect("failed to deserialize")
            })
            .unwrap_or(1.0);

        let angular_velocity_variance = config
            .and_then(|c| {
                c.get::<f64>("t265_angular_velocity_variance")
                    .expect("failed to deserialize")
            })
            .unwrap_or(1.0);

        let mut manager = T265Manager::new().expect("Failed to create t265 manager");
        manager
            .discover_devices_with_options(true)
            .expect("Failed to discover devices with auto boot");
        manager
            .enable_all_video_streams()
            .expect("failed to enable video streams");

        let imu_rx = manager
            .start_all_imu_streams()
            .expect("failed to start imu stream");
        let pose_rx = manager
            .start_all_pose_streams()
            .expect("failed to start pose stream");
        let image_rx = manager
            .start_all_video_streams()
            .expect("failed to start image stream");
        const WIDTH: usize = 848;
        const HEIGHT: usize = 800;
        let pool = CuHostMemoryPool::new("t265_imgs", 4, || vec![0u8; (WIDTH * HEIGHT) as usize])?;

        Ok(Self {
            last_seen: 0,
            initial_yaw_offset: None,
            velocity_variance,
            angular_velocity_variance,
            manager,
            imu_rx,
            pose_rx,
            image_rx,
            pool,
        })
    }

    fn process<'o>(
        &mut self,
        clock: &cu29::prelude::RobotClock,
        new_msg: &mut Self::Output<'o>,
    ) -> cu29::CuResult<()> {
        new_msg.0.clear_payload();
        new_msg.1.clear_payload();

        if let Ok(image) = self.image_rx.try_recv() {
            use cu_sensor_payloads::CuImageBufferFormat;

            let Some(handle) = self.pool.acquire() else {
                return Err(CuError::from("No handle available for t265"));
            };
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
        if let Ok(Pose {
            translation,
            rotation,
            velocity: _,
            angular_velocity: _,
            acceleration,
            angular_acceleration,
            timestamp_ns: _,
            tracker_confidence,
            mapper_confidence: _,
            tracker_state: _,
            device_id,
        }) = self.pose_rx.try_recv()
        {
            // T265 to robot coordinate frame

            use t265_rs::Confidence;
            let transformed_translation =
                Vector3::new(-translation[2], translation[0], translation[1]);
            let t265_translation = Vector3::new(
                transformed_translation.x,
                -transformed_translation.y,
                transformed_translation.z,
            );

            let (qx, qy, qz, qw) = (rotation[0], rotation[1], rotation[2], rotation[3]);
            let t265_rotation =
                UnitQuaternion::new_normalize(nalgebra::Quaternion::new(qw, -qz, -qx, qy));

            let t265_pose = Isometry3::from_parts(t265_translation.into(), t265_rotation);

            // t264 starts with a basically arbitrary "twist" error
            if self.initial_yaw_offset.is_none() {
                let initial_yaw = t265_pose.rotation.euler_angles().2;
                self.initial_yaw_offset = Some(initial_yaw);
            }

            // align with world frame (robot starts facing +X)
            let yaw_correction = UnitQuaternion::from_axis_angle(
                &Vector3::z_axis(),
                -self.initial_yaw_offset.unwrap(),
            );

            let corrected_robot_pose =
                yaw_correction * Isometry3::from_parts(t265_pose.translation, t265_pose.rotation);
            let (velocity_variance, angular_velocity_variance) =
                if tracker_confidence == Confidence::High {
                    (self.velocity_variance, self.angular_velocity_variance)
                } else if tracker_confidence == Confidence::Medium {
                    (
                        self.velocity_variance * 2.,
                        self.angular_velocity_variance * 2.,
                    )
                } else if tracker_confidence == Confidence::Low {
                    (
                        self.velocity_variance * 5.,
                        self.angular_velocity_variance * 5.,
                    )
                } else {
                    (
                        self.velocity_variance * 30.,
                        self.angular_velocity_variance * 30.,
                    )
                };
            let transformed_accel: Vector3<f32> =
                yaw_correction * Vector3::new(acceleration[2], -acceleration[0], acceleration[1]);
            let transformed_angluar_accel = yaw_correction
                * Vector3::new(
                    angular_acceleration[2],
                    -angular_acceleration[0],
                    angular_acceleration[1],
                );

            let imu_msg = T265IMUMsg {
                accel: transformed_accel.data.0[0],
                angular_accel: transformed_angluar_accel.data.0[0],
            };
            let payload = T265Msg {
                pose: EncodableIsometry::from_na(&corrected_robot_pose.cast::<f64>()),
                velocity_variance: velocity_variance,
                angular_velocity_variance: angular_velocity_variance,
                node_name: device_id,
                imu_msg,
            };
            new_msg.0.set_payload(payload);
            self.last_seen = clock.now().as_nanos();
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
    type Output<'m> = output_msg!(T265Msg);
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
