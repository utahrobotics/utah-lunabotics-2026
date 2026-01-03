use std::collections::HashMap;
use std::f64::consts::PI;

use bincode::{Decode, Encode};
use cu_apriltag::AprilTagDetections;
use cu_spatial_payloads::EncodableIsometry;
use cu29::cutask::CuMsg;
use cu29::{
    CuResult, clock::RobotClock, config::ComponentConfig, cutask::Freezable, input_msg, prelude::*,
};

use nalgebra::{SMatrix, SVector};
use ron::de::from_str as ron_from_str;
use serde::Deserialize;
use std::fs;
use std::path::Path;

use crate::ROBOT_STATE;
use crate::rerun_viz::RECORDER;


const LATERAL_VARIANCE_MODIFIER: f64 = 0.001;
const DEPTH_VARIENCE_MODIFIER: f64 = 0.005;
const ANGULAR_VARIANCE_MODIFIER: f64 = 0.005;


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
    max_distance: f64
}

#[derive(Clone, Copy, Debug, Encode, Decode, Serialize)]
#[repr(C)]
pub struct AprilTagMeasurement {
    /// Estimated isometry of the robot base
    pub estimated_isometry: EncodableIsometry,
    #[serde(serialize_with = "<[_]>::serialize")]
    pub variance: [f64; 36]
}

impl Default for AprilTagMeasurement {
    fn default() -> Self {
        Self {
            estimated_isometry: EncodableIsometry::default(),
            variance: [0.0; 36]
        }
    }
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

    fn new(config: Option<&ComponentConfig>) -> CuResult<Self> {
        let known_tags = load_known_apriltag_isometries()?;
        let max_distance = config.expect("provide a config for apriltag handler").get("max_distance").unwrap_or(1.0);

        Ok(Self { known_tags, max_distance })
    }

    fn process(
        &mut self,
        _clock: &RobotClock,
        input: &Self::Input<'_>,
        output: &mut Self::Output<'_>,
    ) -> CuResult<()> {
        output.clear_payload();
        let (input1, input2, input3) = input;

        let mut result_vec: Vec<AprilTagMeasurement> = Vec::new();

        for particular_input in [input1, input2, input3] {
            if let Some(dets) = particular_input.payload() {
                let camera_id = dets.camera_id.as_ref().clone();
                for observation in self.cu_detections_to_tag_observations(dets, &camera_id) {
                    let distrust = observation.decision_margin * observation.decision_margin;
                    let Some(isometry) = observation.get_isometry_of_observer() else {
                        eprintln!("tag observed by unknown camera. (make sure the camera node is correct and defined in the chain");
                        continue;
                    };
                    if observation.tag_local_isometry.translation.vector.magnitude() > self.max_distance {
                        println!("ignored > max distance");
                        continue;
                    }


                    // Translational
                    let position_state: SVector<f64, 3> = isometry.translation.vector;

                    // Make covariance matrix. The eigenvalues associated with eigenvectors of 
                    // a covarience matrix describe its variance in the directions of those eigenvectors.
                    // So, a matrix is constructed from orthogonal eigenvectors, and chosen eigenvalues
                    // This is done with the technique found here (https://math.stackexchange.com/questions/1261462/how-to-reconstruct-a-symmetric-matrix-given-the-eigenvalues-and-eigenvectors)
                    let lateral_vector_1 = position_state.cross(&SVector::<f64, 3>::new(0.0, 0.0, 1.0));
                    let lateral_vector_2 = position_state.cross(&lateral_vector_1);

                    let translational_eigenvector_matrix = SMatrix::<f64, 3, 3>::from_columns(&[position_state, lateral_vector_1, lateral_vector_2]);

                    let translational_eigenvalue_matrix = SMatrix::<f64, 3, 3>::from_diagonal(&SVector::<f64, 3>::new(
                        DEPTH_VARIENCE_MODIFIER, 
                        LATERAL_VARIANCE_MODIFIER, 
                        LATERAL_VARIANCE_MODIFIER
                    ));

                    let position_base_covariance_matrix = 
                        translational_eigenvector_matrix * 
                        translational_eigenvalue_matrix * 
                        translational_eigenvector_matrix.try_inverse().expect("April tag eigenvector matrix was not invertable. Likely a cross product edge case.")
                    ;

                    // Angular
                    let orientation_state = 
                        isometry.rotation.axis()
                            .and_then(|vec| Some(vec.into_inner()))
                            .unwrap_or(SVector::<f64, 3>::zeros())
                        * isometry.rotation.angle()
                    ;

                    let orientation_base_covarience_matrix = SMatrix::<f64, 3, 3>::from_diagonal(&SVector::<f64, 3>::new(
                        ANGULAR_VARIANCE_MODIFIER,
                        ANGULAR_VARIANCE_MODIFIER,
                        ANGULAR_VARIANCE_MODIFIER
                    ));

                    // Combination and final prep
                    let position = [
                        position_state.x, 
                        position_state.y,
                        position_state.z,
                    ];
                    let orientation = [
                        orientation_state.x,
                        orientation_state.y,
                        orientation_state.z,
                    ];

                    let mut combined_covariance = [0.0; 36];

                    for i in 0..3 {
                        for j in 0..3 {
                            combined_covariance[6*i + j] = position_base_covariance_matrix[3*i + j] * distrust as f64;
                        }
                    }

                    for i in 0..3 {
                        for j in 0..3 {
                            combined_covariance[6*(i+3) + (j+3)] = orientation_base_covarience_matrix[3*i + j] * distrust as f64;
                        }
                    }

                    result_vec.push(AprilTagMeasurement { 
                        estimated_isometry: EncodableIsometry::from_na(
                            &Isometry3::from_parts(position.into(), 
                            UnitQuaternion::from_scaled_axis(orientation.into()))), 
                        variance: combined_covariance
                    });
                }
            }
        }
        
        if !result_vec.is_empty() {
            output.set_payload(result_vec);
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


    fn cu_detections_to_tag_observations(
        &self,
        dets: &AprilTagDetections,
        camera_id: &str,
    ) -> Vec<TagObservation> {
        let mut apriltags = Vec::new();
        for (id, pose, _) in dets.filtered_by_decision_margin(60.0) {
            if !self.known_tags.contains_key(&id) {
                eprintln!("apriltag {id} seen but not defined in list of known apriltags");
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



use nalgebra::{
    Isometry3, Rotation3, UnitQuaternion,
    Vector3,
};
use rerun::{Boxes3D};

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
    /// Computes T_world_base: the robot base pose in arena/world coordinates.
    /// Uses: world tag position * (tag observation)^-1 * (camera mount)^-1
    /// Returns None if camera node doesn't exist.
    pub fn get_isometry_of_observer(&self) -> Option<Isometry3<f64>> {
        let camera_node = ROBOT_STATE.get()?.kinematic_root.get_node_with_name(&self.camera_id)?;

        // inverse of the camera isometry relative to robot base
        let camera_isometry_inverse = camera_node.get_isometry_from_base().inverse();


        // tag local isometry is the observed location of the tag as if the camera was always at 0,0,0 facing forward
        let inv_rotation = self.tag_local_isometry.rotation.inverse();

        // this should be the isometry of the camera relative to the apriltags position.
        let isometry_of_observer_in_camera_frame = self.tag_global_isometry
            * Isometry3::from_parts(
                (inv_rotation * -self.tag_local_isometry.translation.vector).into(),
                inv_rotation,
            );
        
        // this should be the position of the robot base
        Some(isometry_of_observer_in_camera_frame * camera_isometry_inverse)
    }
}
