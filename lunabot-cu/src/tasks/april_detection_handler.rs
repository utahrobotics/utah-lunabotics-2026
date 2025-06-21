use std::collections::HashMap;

use chrono::SubsecRound;
use cu29::cutask::CuMsg;
use cu29::{
    clock::RobotClock,
    config::ComponentConfig,
    cutask::{CuSinkTask, Freezable},
    input_msg,
    prelude::*,
    CuResult,
};
use cu_apriltag::AprilTagDetections;

use gstreamer::meta::tags;
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
    // Resolve path relative to CWD. Users may run the binary from the crate root
    // so we first try `./apriltag_isometries`, but also fall back to the
    // directory beside the executable (`CARGO_MANIFEST_DIR`).
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

            // Compute the orientation using the same logic as `Apriltag::get_quat`.
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

// Sink that logs AprilTag detections
#[derive(Default)]
pub struct AprilDetectionHandler {
    known_tags: HashMap<usize, Isometry3<f64>>,
}

impl Freezable for AprilDetectionHandler {}

impl<'cl> CuTask<'cl> for AprilDetectionHandler {
    // one detections struct per camera
    type Input = (
        input_msg!('cl, AprilTagDetections),
        input_msg!('cl, AprilTagDetections),
    );

    type Output = output_msg!('cl, Transform3D<f64>);

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        let known_tags = load_known_apriltag_isometries()?;
        Ok(Self { known_tags })
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input, output: Self::Output) -> CuResult<()> {
        let (input1, input2) = input;

        let mut tag_observations: Vec<TagObservation> = Vec::new();
        if let Some(dets) = input1.payload() {
            let mut tags = cu_detections_to_tag_observations(dets);
            tag_observations.append(&mut tags);
            // estimated_transformations.push(self.handle_detections(&tag_observations)?);
        }
        if let Some(dets) = input2.payload() {
            let mut tags = cu_detections_to_tag_observations(dets);
            tag_observations.append(&mut tags);
        }
        if !tag_observations.is_empty() {
            let observer_iso: Transform3D<f64> = self.handle_detections(&tag_observations)?;
            output.set_payload(observer_iso);
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
    fn handle_detections(&self, dets: &[TagObservation]) -> CuResult<Transform3D<f64>> {
        let mut observer_isometries = Vec::new();
        for observation in dets {
            if !self.known_tags.contains_key(&observation.tag_id) {
                continue;
            }
            let location = (
                observation.tag_global_isometry.translation.x as f32,
                observation.tag_global_isometry.translation.y as f32,
                observation.tag_global_isometry.translation.z as f32,
            );
            let seen_at = chrono::Local::now().time().trunc_subsecs(0);
            let quaterion = observation
                .tag_global_isometry
                .rotation
                .quaternion()
                .as_vector()
                .iter()
                .map(|val| *val as f32)
                .collect::<Vec<f32>>();
            if let Err(_e) = rerun_viz::RECORDER.get().unwrap().recorder.log(
                format!("apriltags/{}/location", observation.tag_id),
                &Boxes3D::from_centers_and_half_sizes([(location)], [(0.1, 0.1, 0.01)])
                    .with_quaternions([[
                        quaterion[0],
                        quaterion[1],
                        quaterion[2],
                        quaterion[3],
                    ]])
                    .with_labels([format!("{}", seen_at)]),
            ) {
                error!("Couldn't log april tag: {e}")
            }
            let isometry_of_observer = observation.get_isometry_of_observer();
            observer_isometries.push(isometry_of_observer);
        }

        // Combine all observer isometries into one estimate and return
        let combined = combine_isometries(&observer_isometries);
        let transform = Transform3D::from_na(combined);
        Ok(transform)
    }
}

// Helper to combine a list of isometries by averaging translation and quaternion
fn combine_isometries(isometries: &[Isometry3<f64>]) -> Isometry3<f64> {
    use nalgebra::{Quaternion, UnitQuaternion, Translation3, Vector3};
    if isometries.is_empty() {
        return Isometry3::identity();
    }
    let mut avg_translation = Vector3::zeros();
    let mut avg_quat = Quaternion::new(0.0, 0.0, 0.0, 0.0);
    for iso in isometries {
        avg_translation += iso.translation.vector;
        let q = iso.rotation.quaternion();
        avg_quat += Quaternion::new(q.w, q.i, q.j, q.k);
    }
    avg_translation /= isometries.len() as f64;
    avg_quat /= isometries.len() as f64;
    let avg_quat = avg_quat.normalize();
    let rotation = UnitQuaternion::from_quaternion(avg_quat);
    let translation = Translation3::from(avg_translation);
    Isometry3::from_parts(translation, rotation)
}

// Helper trait to convert Isometry3<f64> to Transform3D<f64>
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
        Transform3D { mat }
    }
}

fn cu_detections_to_tag_observations(dets: &AprilTagDetections) -> Vec<TagObservation> {
    let mut apriltags = Vec::new();
    for (id, pose, _) in dets.filtered_by_decision_margin(60.0) {
        let mut tag_local_isometry = pose.to_na();
        tag_local_isometry.translation.y *= -1.0;
        tag_local_isometry.translation.z *= -1.0;
        let mut scaled_axis = tag_local_isometry.rotation.scaled_axis();
        scaled_axis.y *= -1.0;
        scaled_axis.z *= -1.0;
        tag_local_isometry.rotation = UnitQuaternion::from_scaled_axis(scaled_axis);
        tag_local_isometry.rotation = UnitQuaternion::from_scaled_axis(
            tag_local_isometry.rotation * Vector3::new(0.0, std::f64::consts::PI, 0.0)
        ) * tag_local_isometry.rotation;

        apriltags.push(TagObservation {
            tag_local_isometry,
            tag_global_isometry: Isometry3::identity(),
            decision_margin: 0.0,
            tag_id: id,
        });
    }
    apriltags
}

use cu_spatial_payloads::Transform3D;
use nalgebra::{Isometry3, MatrixView3, MatrixView3x1, Point3, Translation3, UnitQuaternion, Vector3};
use rerun::Boxes3D;

pub trait PoseExt {
    fn to_na(&self) -> Isometry3<f64>;
}

impl PoseExt for Transform3D<f32> {

    fn to_na(&self) -> Isometry3<f64> {

        let rotation: [[f32; 3];3]= self.rotation().map(
            |x| {
                [x[0].value, x[1].value, x[2].value]
            }
        );

        let translation: [f32; 3] = self.translation().map(
            |x| {
                x.value
            }
        );


        let rotation =
            UnitQuaternion::from_matrix(
                &MatrixView3::from_slice(
                    rotation.iter().flatten().map(
                        |x| *x as f64
                    ).collect::<Vec<_>>().as_slice()
                ).transpose()
            );
            

        let translation: Translation3<f64> = MatrixView3x1::from_slice(&translation.map(|x| x as f64))
            .into_owned()
            .into();


        Isometry3::from_parts(translation, rotation)

    }

}

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
#[derive(Clone, Copy)]
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
}

impl std::fmt::Debug for TagObservation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PoseObservation")
            .field("pose", &self.tag_local_isometry)
            .field("decision_margin", &self.decision_margin)
            .finish()
    }
}

impl TagObservation {
    /// Get the isometry of the observer.
    pub fn get_isometry_of_observer(&self) -> Isometry3<f64> {
        let inv_rotation = self.tag_local_isometry.rotation.inverse();
        self.tag_global_isometry
            * Isometry3::from_parts(
                (inv_rotation * -self.tag_local_isometry.translation.vector).into(),
                inv_rotation,
            )
    }
}
