use cu29::{clock::RobotClock, config::ComponentConfig, cutask::{CuMsg, CuSinkTask, Freezable}, input_msg, prelude::*, CuResult};
use cu_spatial_payloads::Transform3D;
use nalgebra::{Isometry3, UnitQuaternion, Vector3};
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use crate::{rerun_viz, utils::{lerp, swing_twist_decomposition}};
use std::collections::HashMap;

// Constants from the old localizer
const ACCELEROMETER_LERP_SPEED: f64 = 150.0;
const LOCALIZATION_DELTA: f64 = 1.0 / 60.0;

pub struct CuLocalizer {
    pub root_node: StaticNode,
}

impl Freezable for CuLocalizer {}

impl<'cl> CuSinkTask<'cl> for CuLocalizer {
    /// HashMap of camera IDs to estimated observer isometry from apriltags
    type Input = input_msg!('cl, Box<HashMap<String, Transform3D<f64>>>);

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self>
        where Self: Sized 
    {
        let layout_path: String = config.unwrap().get("layout_path").unwrap();
        let robot_chain = NodeSerde::from_reader(
            std::fs::File::open(layout_path).expect("Failed to read robot chain"),
        ).expect("Failed to parse robot chain");
        let robot_chain = ChainBuilder::from(robot_chain).finish_static();
        Ok(CuLocalizer {
            root_node: robot_chain,
        })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: Self::Input,
    ) -> CuResult<()> {
        // Get current robot isometry
        let mut isometry = self.root_node.get_global_isometry();
        
        // Check for NaN/infinite values and return error if found
            if isometry.translation.x.is_nan()
                || isometry.translation.y.is_nan()
                || isometry.translation.z.is_nan()
            {
            return Err(CuError::new_with_cause(
                "Robot origin is NaN",
                std::io::Error::new(std::io::ErrorKind::InvalidData, "Robot origin contains NaN values")
            ));
            } else if isometry.translation.x.is_infinite()
                || isometry.translation.y.is_infinite()
                || isometry.translation.z.is_infinite()
            {
            return Err(CuError::new_with_cause(
                "Robot origin is infinite",
                std::io::Error::new(std::io::ErrorKind::InvalidData, "Robot origin contains infinite values")
            ));
            } else if isometry.rotation.w.is_nan()
                || isometry.rotation.i.is_nan()
                || isometry.rotation.j.is_nan()
                || isometry.rotation.k.is_nan()
            {
            return Err(CuError::new_with_cause(
                "Robot rotation is NaN",
                std::io::Error::new(std::io::ErrorKind::InvalidData, "Robot rotation contains NaN values")
            ));
            } else if isometry.rotation.w.is_infinite()
                || isometry.rotation.i.is_infinite()
                || isometry.rotation.j.is_infinite()
                || isometry.rotation.k.is_infinite()
            {
            return Err(CuError::new_with_cause(
                "Robot rotation is infinite",
                std::io::Error::new(std::io::ErrorKind::InvalidData, "Robot rotation contains infinite values")
            ));
        }

        // Process camera-specific apriltag input if available
        if let Some(camera_transforms) = input.payload() {
            let mut all_observer_isometries = Vec::new();
            
            // Process each camera's detections
            for (camera_id, transform) in camera_transforms.iter() {
                // Get the camera node from the robot chain
                if let Some(camera_node) = self.root_node.get_node_with_name(camera_id) {
                    // Get the camera's isometry in the robot frame
                    let mut camera_isometry = camera_node.get_isometry_from_base();
                    // Create inverse to transform from camera frame to robot frame
                    camera_isometry.inverse_mut();
                    
                    // Convert the camera's observer isometry to robot frame
                    let camera_observer_iso: Isometry3<f64> = transform.into();
                    let robot_frame_observer_iso = camera_observer_iso * camera_isometry;
                    
                    all_observer_isometries.push(robot_frame_observer_iso);
                } else {
                    return Err(CuError::new_with_cause( &format!("Camera node '{}' not found in robot chain", camera_id),
                    std::io::Error::new(std::io::ErrorKind::NotFound, "Camera node not found")));
                }
            }
            
            if !all_observer_isometries.is_empty() {
                // Average all camera observations
                let combined_observer_iso = if all_observer_isometries.len() == 1 {
                    all_observer_isometries[0]
                } else {
                    // Simple averaging - could be improved with weighted averaging
                    let mut sum_translation = Vector3::zeros();
                    for iso in &all_observer_isometries {
                        sum_translation += iso.translation.vector;
                    }
                    let mean_translation = sum_translation / all_observer_isometries.len() as f64;
                    
                    // For rotation, just use the first one for now (proper quaternion averaging is complex)
                    let mean_rotation = all_observer_isometries[0].rotation;
                    
                    Isometry3::from_parts(mean_translation.into(), mean_rotation)
                };
                
                // Down axis for swing-twist decomposition (assuming Y-up coordinate system)
                let down_axis = -Vector3::y_axis();
                
                // Lerp the translation (ported from old localizer)
                isometry.translation.vector = lerp(
                    isometry.translation.vector,
                    combined_observer_iso.translation.vector,
                    LOCALIZATION_DELTA,
                    ACCELEROMETER_LERP_SPEED,
                );

                // Handle rotation using swing-twist decomposition (ported from old localizer)
                let (_, new_twist) = swing_twist_decomposition(&combined_observer_iso.rotation, &down_axis);
                let (old_swing, _) = swing_twist_decomposition(&isometry.rotation, &down_axis);
                let new_rotation = old_swing * new_twist;
                
                if new_rotation.w.is_finite()
                    && new_rotation.i.is_finite()
                    && new_rotation.j.is_finite()
                    && new_rotation.k.is_finite()
                {
                    let dot_product = isometry.rotation.coords.dot(&new_rotation.coords);
                    
                    let target_quat = if dot_product < 0.0 {
                        UnitQuaternion::new_normalize(-new_rotation.into_inner())
                    } else {
                        new_rotation
                    };
                    
                    // Use lerp for the quaternion interpolation with proper direction
                    isometry.rotation = UnitQuaternion::new_normalize(lerp(
                        isometry.rotation.into_inner(),
                        target_quat.into_inner(),
                        LOCALIZATION_DELTA,
                        ACCELEROMETER_LERP_SPEED,
                    ));
                }
                
                // Update the robot isometry
                self.root_node.set_isometry(isometry);
                
                // Log to rerun (same pattern as the old localizer)
                if let Some(recorder) = rerun_viz::RECORDER.get() {
                    if let Err(e) = recorder.recorder.log(
                        rerun_viz::ROBOT_STRUCTURE,
                        &rerun::Transform3D::from_translation_rotation(
                            isometry.translation.vector.cast::<f32>().data.0[0],
                            rerun::Quaternion::from_xyzw(
                                isometry.rotation.as_vector().cast::<f32>().data.0[0],
                            ),
                        ),
                    ) {
                        return Err(CuError::new_with_cause(
                            &format!("Failed to log robot transform: {e}"),
                            std::io::Error::new(std::io::ErrorKind::Other, "Rerun logging failed")
                        ));
                    }
                }
            }
        }

        Ok(())
    }
}