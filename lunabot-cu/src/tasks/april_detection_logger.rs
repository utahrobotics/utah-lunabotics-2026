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

// Sink that logs AprilTag detections
#[derive(Default)]
pub struct DetectionLogger;

impl Freezable for DetectionLogger {}

impl<'cl> CuSinkTask<'cl> for DetectionLogger {
    // one detections struct per camera
    type Input = (
        input_msg!('cl, AprilTagDetections),
        input_msg!('cl, AprilTagDetections),
    );

    fn new(_config: Option<&ComponentConfig>) -> CuResult<Self> {
        Ok(Self)
    }

    fn process(&mut self, _clock: &RobotClock, input: Self::Input) -> CuResult<()> {
        let (input1, input2) = input;

        if let Some(dets) = input1.payload() {
            let tag_observations = cu_detections_to_tag_observations(dets);
            for tag_observation in tag_observations {
                info!(
                    "[{}] Detected tag {} with pose: {}",
                    &dets.camera_id, tag_observation.tag_id, tag_observation.tag_local_isometry
                );
            }
        }
        if let Some(dets) = input2.payload() {
            let tag_observations = cu_detections_to_tag_observations(dets);
            for tag_observation in tag_observations {
                //TODO: poses still are garbage from that one godforsaken bug
                info!(
                    "[{}] Detected tag {} with pose: {}",
                    &dets.camera_id, tag_observation.tag_id, tag_observation.tag_local_isometry
                );
            }
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
use serde::Deserialize;


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
        // let mut observer_pose = self.tag_local_isometry;
        // observer_pose.translation.vector = self.tag_global_isometry.translation.vector
        //     + self.tag_global_isometry.rotation
        //         * observer_pose.rotation.inverse()
        //         * observer_pose.translation.vector;
        // observer_pose.rotation = self.tag_global_isometry.rotation
        //     * UnitQuaternion::from_axis_angle(&(observer_pose.rotation * Vector3::y_axis()), PI)
        //     * observer_pose.rotation;
        // observer_pose
        let inv_rotation = self.tag_local_isometry.rotation.inverse();
        self.tag_global_isometry
            * Isometry3::from_parts(
                (inv_rotation * -self.tag_local_isometry.translation.vector).into(),
                inv_rotation,
            )
    }
}
