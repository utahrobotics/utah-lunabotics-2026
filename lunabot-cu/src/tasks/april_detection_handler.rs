use std::collections::HashMap;
use std::f64::consts::PI;

use cu_apriltag::AprilTagDetections;
use cu29::cutask::CuMsg;
use cu29::{
    CuResult, clock::RobotClock, config::ComponentConfig, cutask::Freezable, input_msg, prelude::*,
};

use cu_spatial_payloads::EncodableIsometry;
use ron::de::from_str as ron_from_str;
use serde::Deserialize;
use std::fs;
use std::path::Path;

use crate::rerun_viz::RECORDER;

/// Data definition that mirrors the contents of a `.ron` apriltag isometry file.
/// The field names are intentionally kept simple so that we can be flexible with
/// the on-disk representation without changing the runtime `Apriltag` struct.
#[derive(Deserialize)]
struct TagDef {
    tag_id: usize,
    origin: (f64, f64, f64),
    // roll pitch yaw
    euler: (f64, f64, f64),
}

impl TagDef {
    pub fn get_quat(self) -> UnitQuaternion<f64> {
        from_euler_angles(self.euler.0, self.euler.1, self.euler.2)
    }
}

fn from_euler_angles(roll: f64, pitch: f64, yaw: f64) -> UnitQuaternion<f64> {
    let roll_rad = roll.to_radians();
    let pitch_rad = pitch.to_radians();
    let yaw_rad = yaw.to_radians();

    let rot_z = UnitQuaternion::from_scaled_axis(Vector3::z() * yaw_rad);
    let rot_y = UnitQuaternion::from_scaled_axis(Vector3::y() * pitch_rad);
    let rot_x = UnitQuaternion::from_scaled_axis(Vector3::x() * roll_rad);

    rot_z * rot_y * rot_x
}

/// Reads every `*.ron` file in `apriltag_isometries/`
///  and returns a mapping from tag ID to its global `Isometry3`.
fn load_known_apriltag_isometries() -> CuResult<HashMap<usize, Isometry3<f64>>> {
    let search_paths = [
        Path::new("apriltag_isometries").to_path_buf(),
        Path::new(env!("CARGO_MANIFEST_DIR")).join("apriltag_isometries"),
    ];

    let mut known_tags = HashMap::new();

    for dir in &search_paths {
        if !dir.exists() {
            continue;
        }

        for entry in fs::read_dir(dir).map_err(|e| e.to_string())? {
            let entry = entry.map_err(|e| e.to_string())?;
            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) != Some("ron") {
                continue;
            }

            let contents = fs::read_to_string(&path).map_err(|e| e.to_string())?;
            let def: TagDef = ron_from_str(&contents).map_err(|e| e.to_string())?;

            let translation = Vector3::new(def.origin.0, def.origin.1, def.origin.2);
            let id = def.tag_id;
            let rotation = def.get_quat();

            known_tags.insert(id, Isometry3::from_parts(translation.into(), rotation));
        }

        if !known_tags.is_empty() {
            break;
        }
    }

    Ok(known_tags)
}

#[derive(Default)]
pub struct AprilDetectionHandler {
    known_tags: HashMap<usize, Isometry3<f64>>,
}

#[derive(Clone, Copy, Default, Debug, Encode, Decode, Serialize, ZeroCopySend)]
#[repr(C)]
pub struct AprilTagMeasurement {
    position: [f64; 3],
    orientation: [f64; 3],
    variance: [f64; 36]
}

impl Freezable for AprilDetectionHandler {}

impl CuTask for AprilDetectionHandler {
    // one detections struct per camera
    type Input<'m> = (
        &'m input_msg!(AprilTagDetections),
        &'m input_msg!(AprilTagDetections),
        &'m input_msg!(AprilTagDetections),
    );
    // camera_id, estimated isometry of camera
    type Output<'m> = output_msg!(Vec<AprilTagMeasurement>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        let known_tags = load_known_apriltag_isometries()?;
        Ok(Self { known_tags })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.clear_payload();
        let (input1, input2, input3) = input;

        let mut result_map = HashMap::new();

        if let Some(dets) = input1.payload() {
            let camera_id = dets.camera_id.as_ref().clone();
            let tags = self.cu_detections_to_tag_observations(dets, &camera_id);
            if !tags.is_empty() {
                let observer_iso = self.estimate_observer_isometry_from_observations(&tags)?;
                result_map.insert(camera_id, EncodableIsometry::from_na(&observer_iso));
            }
        }

        if let Some(dets) = input2.payload() {
            let camera_id = dets.camera_id.as_ref().clone();
            let tags = self.cu_detections_to_tag_observations(dets, &camera_id);
            if !tags.is_empty() {
                let observer_iso = self.estimate_observer_isometry_from_observations(&tags)?;
                result_map.insert(camera_id, EncodableIsometry::from_na(&observer_iso));
            }
        }

        if let Some(dets) = input3.payload() {
            let camera_id = dets.camera_id.as_ref().clone();
            let tags = self.cu_detections_to_tag_observations(dets, &camera_id);
            if !tags.is_empty() {
                let observer_iso = self.estimate_observer_isometry_from_observations(&tags)?;
                result_map.insert(camera_id, EncodableIsometry::from_na(&observer_iso));
            }
        }
        if !result_map.is_empty() {
            output.set_payload(Box::new(result_map));
        }
        Ok(())
    }

    fn start(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn preprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn postprocess(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }

    fn stop(&mut self, _clock: &RobotClock) -> CuResult<()> {
        Ok(())
    }
}

impl AprilDetectionHandler {
    fn transform_cv_to_physics(cv_pose: Isometry3<f64>) -> Isometry3<f64> {
        let cv_translation = cv_pose.translation.vector;
        let cv_x = cv_translation[0];
        let cv_y = cv_translation[1];
        let cv_z = cv_translation[2];

        let physics_x = cv_z;
        let physics_y = -cv_x;
        let physics_z = -cv_y;

        let physics_translation = Vector3::new(physics_x, physics_y, physics_z);

        let cv_rotation_matrix = cv_pose.rotation.to_rotation_matrix().matrix().clone();

        let coord_transform = nalgebra::Matrix3::new(0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0);

        let physics_rotation_matrix =
            coord_transform * cv_rotation_matrix * coord_transform.transpose();
        let physics_rotation = UnitQuaternion::from_rotation_matrix(
            &Rotation3::from_matrix_unchecked(physics_rotation_matrix),
        );

        Isometry3::from_parts(physics_translation.into(), physics_rotation)
    }

    /// estimates the observers isometry from each observation individually, and then averages them all together
    /// additionally logs the tags known global coords to rerun with a timestamp so we know the tag has been seen recently.
    /// filters out tags that are more than 3m away
    fn estimate_observer_isometry_from_observations(
        &self,
        observations: &[TagObservation],
    ) -> CuResult<Isometry3<f64>> {
        let mut observer_isometries = Vec::new();

        for observation in observations {
            let isometry_of_observer = observation.get_isometry_of_observer();

            if isometry_of_observer
                .translation
                .vector
                .metric_distance(&observation.tag_global_isometry.translation.vector)
                > 3.0
            {
                println!(
                    "apriltag too far away, distance: {}",
                    isometry_of_observer
                        .translation
                        .vector
                        .metric_distance(&observation.tag_global_isometry.translation.vector)
                );
                continue;
            }
            observer_isometries.push(isometry_of_observer);
        }

        if observer_isometries.is_empty() {
            return Err("No valid tag observations".into());
        }

        let combined = combine_isometries(&observer_isometries);

        Ok(combined)
    }

    fn cu_detections_to_tag_observations(
        &self,
        dets: &AprilTagDetections,
        camera_id: &str,
    ) -> Vec<TagObservation> {
        let mut apriltags = Vec::new();
        for (id, pose, _) in dets.filtered_by_decision_margin(60.0) {
            if !self.known_tags.contains_key(&id) {
                continue;
            }

            let Some(pose) = pose.to_na() else {
                warning!("failed to convert pose to nalgebra type");
                continue;
            };

            let tag_local_isometry: Isometry3<f64> = Self::transform_cv_to_physics(pose);
            let tag_global_isometry = self.known_tags.get(&id).unwrap();

            let tag_half_size = (0.002, 0.08, 0.08);

            RECORDER
                .get()
                .unwrap()
                .recorder
                .log(
                    format!("apriltags/{}/{}/location", camera_id, id),
                    &Boxes3D::from_centers_and_half_sizes(vec![(0., 0., 0.)], vec![tag_half_size])
                        .with_colors([rerun::Color::from_rgb(255, 255, 255)])
                        .with_labels([format!("id: {}", id)]),
                )
                .unwrap();

            RECORDER
                .get()
                .unwrap()
                .recorder
                .log(
                    format!("apriltags/{}/{}/location", camera_id, id),
                    &rerun::Transform3D::from_translation_rotation(
                        tag_global_isometry.translation.vector.cast::<f32>().data.0[0],
                        rerun::Quaternion::from_xyzw(
                            tag_global_isometry
                                .rotation
                                .as_vector()
                                .cast::<f32>()
                                .data
                                .0[0],
                        ),
                    ),
                )
                .unwrap();

            let yaw_180 = UnitQuaternion::from_euler_angles(0., 0.0, PI);

            let tag_global_isometry = Isometry3::from_parts(
                tag_global_isometry.translation,
                yaw_180 * tag_global_isometry.rotation,
            );

            apriltags.push(TagObservation {
                tag_local_isometry,
                tag_global_isometry,
                decision_margin: 0.0,
                tag_id: id,
                camera_id: camera_id.to_string(),
            });
        }
        apriltags
    }
}

/// Helper to combine a list of isometries by averaging translation and quaternion
fn combine_isometries(isometries: &[Isometry3<f64>]) -> Isometry3<f64> {
    if isometries.is_empty() {
        return Isometry3::identity();
    }
    if isometries.len() == 1 {
        return isometries[0];
    }

    let mut sum_translation = Vector3::zeros();
    for isometry in isometries {
        sum_translation += isometry.translation.vector;
    }
    let mean_translation = sum_translation / isometries.len() as f64;

    let mean_rotation = average_quaternions(
        &isometries
            .iter()
            .map(|iso| iso.rotation)
            .collect::<Vec<_>>(),
    );

    Isometry3::from_parts(Translation3::from(mean_translation), mean_rotation)
}

/// This method converts quaternions to rotation matrices, averages them, and converts back
fn average_quaternions(quaternions: &[UnitQuaternion<f64>]) -> UnitQuaternion<f64> {
    if quaternions.is_empty() {
        return UnitQuaternion::identity();
    }
    if quaternions.len() == 1 {
        return quaternions[0];
    }

    let mut sum_matrix = nalgebra::Matrix3::zeros();
    for quat in quaternions {
        sum_matrix += quat.to_rotation_matrix().matrix();
    }
    sum_matrix /= quaternions.len() as f64;

    let svd = sum_matrix.svd(true, true);
    if let (Some(u), Some(v_t)) = (svd.u, svd.v_t) {
        let mut rotation_matrix = u * v_t;
        if rotation_matrix.determinant() < 0.0 {
            let mut u_corrected = u;
            u_corrected.set_column(2, &(-u.column(2)));
            rotation_matrix = u_corrected * v_t;
        }

        UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(
            rotation_matrix,
        ))
    } else {
        // Fallback to component averaging if SVD fails
        average_quaternions_component_based(quaternions)
    }
}

/// Fallback quaternion averaging using component-based approach
/// This handles the quaternion double-cover issue (q and -q represent the same rotation)
fn average_quaternions_component_based(quaternions: &[UnitQuaternion<f64>]) -> UnitQuaternion<f64> {
    if quaternions.is_empty() {
        return UnitQuaternion::identity();
    }
    if quaternions.len() == 1 {
        return quaternions[0];
    }

    // Use the first quaternion as reference for handling double-cover
    let reference = quaternions[0];
    let mut sum = reference.coords;

    for quat in &quaternions[1..] {
        // Handle quaternion double-cover: choose the quaternion representation
        let quat_coords = if reference.coords.dot(&quat.coords) >= 0.0 {
            quat.coords
        } else {
            -quat.coords
        };
        sum += quat_coords;
    }

    // Average and normalize
    let mean_coords = sum / quaternions.len() as f64;
    UnitQuaternion::new_normalize(nalgebra::Quaternion::from(mean_coords))
}

use nalgebra::{
    Isometry3, Matrix3, Matrix4, Quaternion, Rotation, Rotation3, Translation3, UnitQuaternion,
    UnitVector3, Vector3,
};
use rerun::{Archetype, Arrows3D, Boxes3D, Vector3D};

/// An observation of the global orientation and position
/// of the camera that observed an apriltag.
#[derive(Clone)]
pub struct TagObservation {
    /// The orientation and position of the apriltag relative to the observer.
    pub tag_local_isometry: Isometry3<f64>,
    /// The orientation and position of the apriltag in global space.
    ///
    /// These are the same values that were passed to `add_tag`. As such,
    /// if these values were not known then, this value will be incorrect.
    /// However, this can be set to the correct value, allowing
    /// `get_isometry_of_observer` to produce correct results.
    pub tag_global_isometry: Isometry3<f64>,
    /// The goodness of an observation.
    ///
    /// This is a value generated by the apriltag detector.
    pub decision_margin: f32,

    pub tag_id: usize,
    pub camera_id: String,
}

impl std::fmt::Debug for TagObservation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("TagObservation")
            .field("tag_id", &self.tag_id)
            .field("camera_id", &self.camera_id)
            .finish()
    }
}

impl TagObservation {
    pub fn get_isometry_of_observer(&self) -> Isometry3<f64> {
        let inv_rotation = self.tag_local_isometry.rotation.inverse();
        self.tag_global_isometry
            * Isometry3::from_parts(
                (inv_rotation * -self.tag_local_isometry.translation.vector).into(),
                inv_rotation,
            )
    }
}
