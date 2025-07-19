use std::collections::HashMap;

use chrono::SubsecRound;
use cu29::cutask::CuMsg;
use cu29::{
    clock::RobotClock,
    config::ComponentConfig,
    cutask::Freezable,
    input_msg,
    prelude::*,
    CuResult,
};
use cu_apriltag::AprilTagDetections;


use serde::Deserialize;
use std::fs;
use std::path::Path;
use ron::de::from_str as ron_from_str;

use crate::rerun_viz;

/// Data definition that mirrors the contents of a `.ron` apriltag isometry file.
/// The field names are intentionally kept simple so that we can be flexible with
/// the on-disk representation without changing the runtime `Apriltag` struct.
#[derive(Deserialize)]
struct TagDef {
    tag_id: usize,
    origin: (f64, f64, f64),
    forward_axis: (f64, f64, f64),
    #[serde(default)]
    roll: f64,
}

/// Reads every `*.ron` file in `apriltag_isometries/` (located next to the
/// executable when it is run) and returns a mapping from tag ID to its global
/// `Isometry3`.
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

            let (x, y, z) = def.origin;
            let (fx, fy, fz) = def.forward_axis;
            let forward_axis = Vector3::new(fx, fy, fz);

            let rotation1 = UnitQuaternion::rotation_between(&Vector3::new(0.0, 0.0, -1.0), &forward_axis)
                .unwrap_or(UnitQuaternion::from_scaled_axis(Vector3::new(0.0, std::f64::consts::PI, 0.0)));

            let cross_axis = forward_axis.cross(&Vector3::new(0.0, 1.0, 0.0));
            let true_up = cross_axis.cross(&forward_axis);

            let actual_up = rotation1 * Vector3::new(0.0, 1.0, 0.0);
            let rotation2 = UnitQuaternion::rotation_between(&actual_up, &true_up)
                .unwrap_or(UnitQuaternion::identity());

            let rotation3 = UnitQuaternion::from_scaled_axis(forward_axis.normalize() * def.roll);

            let orientation = rotation3 * rotation2 * rotation1;

            let isometry = Isometry3::from_parts(Translation3::new(x, y, z), orientation);

            known_tags.insert(def.tag_id, isometry);
        }

        // Prefer the first directory that exists & contains files.
        if !known_tags.is_empty() {
            break;
        }
    }

    Ok(known_tags)
}

#[derive(Default)]
pub struct AprilDetectionHandler {
    known_tags: HashMap<usize, Isometry3<f64>>,
    process_counter: u32,
}

impl Freezable for AprilDetectionHandler {}

impl<'cl> CuTask<'cl> for AprilDetectionHandler {
    // one detections struct per camera
    type Input = (
        input_msg!('cl, AprilTagDetections),
        input_msg!('cl, AprilTagDetections),
        input_msg!('cl, AprilTagDetections),
    );

    type Output = output_msg!('cl, Box<HashMap<String, Transform3D<f64>>>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        let known_tags = load_known_apriltag_isometries()?;
        Ok(Self { known_tags, process_counter: 0 })
    }

    fn process(&mut self, clock: &RobotClock, input: Self::Input, output: Self::Output) -> CuResult<()> {
        let start = clock.now().as_nanos();
        let (input1, input2, input3) = input;

        let mut result_map = HashMap::new();
        
        if let Some(dets) = input1.payload() {
            let camera_id = dets.camera_id.as_ref().clone();
            let tags = self.cu_detections_to_tag_observations(dets, &camera_id);
            if !tags.is_empty() {
                let observer_iso = self.handle_detections(&tags)?;
                result_map.insert(camera_id, observer_iso);
            }
        }
        
        if let Some(dets) = input2.payload() {
            let camera_id = dets.camera_id.as_ref().clone();
            let tags = self.cu_detections_to_tag_observations(dets, &camera_id);
            if !tags.is_empty() {
                let observer_iso = self.handle_detections(&tags)?;
                result_map.insert(camera_id, observer_iso);
            }
        }

        if let Some(dets) = input3.payload() {
            let camera_id = dets.camera_id.as_ref().clone();
            let tags = self.cu_detections_to_tag_observations(dets, &camera_id);
            if !tags.is_empty() {
                let observer_iso = self.handle_detections(&tags)?;
                result_map.insert(camera_id, observer_iso);
            }
        }
        
        if !result_map.is_empty() {
            output.set_payload(Box::new(result_map));
        }
        let duration = clock.now().as_nanos() - start;

        if duration/1000 > 10 {
            eprintln!("apriltag detection handler took {} us", duration/1000)
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
    fn handle_detections(&self, observations: &[TagObservation]) -> CuResult<Transform3D<f64>> {
        let mut observer_isometries = Vec::new();
        
        for observation in observations {
            // Look up the global pose of this tag.
            let tag_global_isometry = observation.tag_global_isometry;
            
            // Log the tag position in global coordinates for visualization.
            let location = (
                tag_global_isometry.translation.x as f32,
                tag_global_isometry.translation.y as f32,
                tag_global_isometry.translation.z as f32,
            );
            let seen_at = chrono::Local::now().time().trunc_subsecs(0);
            let quaternion_vec = tag_global_isometry
                .rotation
                .quaternion()
                .as_vector()
                .iter()
                .map(|val| *val as f32)
                .collect::<Vec<f32>>();
                
            if let Err(e) = rerun_viz::RECORDER.get().unwrap().recorder.log(
                format!("apriltags/{}/{}/location", observation.camera_id, observation.tag_id),
                &Boxes3D::from_centers_and_half_sizes([(location)], [(0.1, 0.1, 0.01)])
                    .with_quaternions([[
                        quaternion_vec[0],
                        quaternion_vec[1],
                        quaternion_vec[2],
                        quaternion_vec[3],
                    ]])
                    .with_labels([format!("{}", seen_at)]),
            ) {
                return Err(CuError::new_with_cause(
                    &format!("Couldn't log april tag: {e}"),
                    std::io::Error::new(std::io::ErrorKind::Other, "Rerun logging failed")
                ));
            }
            
            // Compute the camera pose from the tag's known global pose and the observed local pose.
            let isometry_of_observer = observation.tag_local_isometry.inverse() * tag_global_isometry;
            observer_isometries.push(isometry_of_observer);
        }
        
        if observer_isometries.is_empty() {
            return Err("No valid tag observations".into());
        }
        
        let combined = combine_isometries(&observer_isometries);
        let transform = Transform3D::from_na(combined);
        Ok(transform)
    }

    fn cu_detections_to_tag_observations(&self, dets: &AprilTagDetections, camera_id: &str) -> Vec<TagObservation> {
        let mut apriltags = Vec::new();
        for (id, pose, _) in dets.filtered_by_decision_margin(60.0) {
            if !self.known_tags.contains_key(&id) {
                continue;
            }
            // Convert pose and flip axes to align with robot coordinate conventions.
            let pose: Transform3D<f64> = pose.cast();
            let mut tag_local_isometry: Isometry3<f64> = (&pose).into();

            // Invert Y and Z translations (camera frame -> robot frame)
            tag_local_isometry.translation.y *= -1.0;
            tag_local_isometry.translation.z *= -1.0;

            // Flip the corresponding rotation components.
            let mut scaled_axis = tag_local_isometry.rotation.scaled_axis();
            scaled_axis.y *= -1.0;
            scaled_axis.z *= -1.0;
            tag_local_isometry.rotation = UnitQuaternion::from_scaled_axis(scaled_axis);

            // Apply an additional 180° rotation around the Y-axis so the tag faces forward.
            tag_local_isometry.rotation = UnitQuaternion::from_scaled_axis(
                tag_local_isometry.rotation * Vector3::new(0.0, std::f64::consts::PI, 0.0),
            ) * tag_local_isometry.rotation;

            apriltags.push(TagObservation {
                tag_local_isometry,
                tag_global_isometry: *self.known_tags.get(&id).unwrap(),
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

    let mean_rotation = average_quaternions(&isometries.iter().map(|iso| iso.rotation).collect::<Vec<_>>());

    Isometry3::from_parts(Translation3::from(mean_translation), mean_rotation)
}

/// Proper quaternion averaging using rotation matrix approach
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
        
        UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(rotation_matrix))
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

/// Helper trait to convert Isometry3<f64> to Transform3D<f64>
trait Transform3DFromNa {
    fn from_na(iso: Isometry3<f64>) -> Self;
}

impl Transform3DFromNa for Transform3D<f64> {
    fn from_na(iso: Isometry3<f64>) -> Self {
        let translation = iso.translation.vector;
        let rotation = iso.rotation.to_rotation_matrix();
        let mut mat = [[0.0; 4]; 4];
        for i in 0..3 {
            for j in 0..3 {
                mat[i][j] = rotation[(i, j)];
            }
        }
        mat[0][3] = translation.x;
        mat[1][3] = translation.y;
        mat[2][3] = translation.z;
        mat[3][0] = 0.0;
        mat[3][1] = 0.0;
        mat[3][2] = 0.0;
        mat[3][3] = 1.0;
        Transform3D::from_matrix(mat)
    }
}

use cu_spatial_payloads::{Transform3D, Transform3DCast};
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};
use rerun::Boxes3D;


#[derive(Deserialize, Clone, Copy)]
pub struct Apriltag {
    pub tag_position: Point3<f64>,
    forward_axis: Vector3<f64>,
    #[serde(default)]
    roll: f64,
    pub tag_width: f64,
}

impl Apriltag {
    pub fn get_quat(self) -> UnitQuaternion<f64> {
        // First rotation to face along the forward axis
        let rotation1 =
            UnitQuaternion::rotation_between(&Vector3::new(0.0, 0.0, -1.0), &self.forward_axis)
                .unwrap_or(UnitQuaternion::from_scaled_axis(Vector3::new(0.0, std::f64::consts::PI, 0.0)));

        let cross_axis = self.forward_axis.cross(&Vector3::new(0.0, 1.0, 0.0));
        let true_up = cross_axis.cross(&self.forward_axis);

        // Second rotation to rotate the up axis to face directly up
        let actual_up = rotation1 * Vector3::new(0.0, 1.0, 0.0);
        let rotation2 = UnitQuaternion::rotation_between(&actual_up, &true_up).unwrap();

        // Third rotation to roll the tag
        let rotation3 = UnitQuaternion::from_scaled_axis(self.forward_axis.normalize() * self.roll);

        rotation3 * rotation2 * rotation1
    }
}

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
    /// Get the isometry of the observer.
    pub fn get_isometry_of_observer(&self) -> Isometry3<f64> {
        self.tag_local_isometry.inverse() * self.tag_global_isometry
    }
}
