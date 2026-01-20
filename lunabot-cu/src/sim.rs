#![feature(try_blocks, f16, mpmc_channel)]
pub mod comms;
pub mod rerun_viz;

pub mod bridges;
pub mod pathfinding;
pub mod robot_state;
pub mod simple_monitor;
pub mod tasks;
pub mod utils;

use common::FromLunabot;
use crossbeam::atomic::AtomicCell;
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use embedded_common::Direction;
use iceoryx_types::IceoryxDepthFrame;
use mujoco_rs::cpp_viewer::MjViewerCpp;
use mujoco_rs::prelude::*;
use mujoco_rs::renderer::MjRendererBuilder;
use nalgebra::{Isometry3, Quaternion, Translation, Translation3, UnitQuaternion, Vector3};
use nalgebra::{SMatrix, SVector};
use rerun::{Arrows3D, Transform3D};
use robot_state::RobotState;
use simple_motion::{ChainBuilder, NodeSerde};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::{OnceLock, RwLock};
use wgsl_pcl::wgsl_setup::{init_gpu_blocking, is_gpu_initialized};

use crate::rerun_viz::{RECORDER, ROBOT_STRUCTURE};
use crate::tasks::{DEPTH_FRAME_HEIGHT, DEPTH_FRAME_WIDTH};

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

pub static ROBOT_STATE: OnceLock<RobotState> = OnceLock::new();

pub static TARGET_HZ: usize = 1000; // MUST BE THE SAME AS THE TARGET HZ IN COPPERCONFIG.RON

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct LunabotApplication {}

fn default_callback(step: default::SimStep) -> SimOverride {
    match step {
        default::SimStep::UdevMonitor(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Pointcloud(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Imu(_) => SimOverride::ExecutedBySim,
        default::SimStep::V3Pico(_) => SimOverride::ExecutedBySim,
        default::SimStep::MotorCtrl(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::RealsenseSubscriber(_) => SimOverride::ExecutedBySim,
        default::SimStep::T265Subscriber(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectionHandler(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2KissIcp(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::OccupancyGridPipeline(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::NewAi(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::Localizer(_) => SimOverride::ExecutedBySim,
        default::SimStep::LunabaseBridgeRxFromLunabaseRx(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::LunabaseBridgeTxToLunabase(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::__Phantom(_) => SimOverride::ExecutedBySim,
    }
}

fn sim_callback(
    step: default::SimStep,
    data: &mut MjData<&MjModel>,
    renderer: &mut mujoco_rs::renderer::MjRenderer<&MjModel>,
) -> SimOverride {
    match step {
        default::SimStep::UdevMonitor(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::CamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::GstConvertLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Pointcloud(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2Imu(_) => SimOverride::ExecutedBySim,
        default::SimStep::V3Pico(CuTaskCallbackState::Process(input, _)) => {
            static LIFT_DIRECTION: AtomicCell<Direction> = AtomicCell::new(Direction::Forward);
            static BUCKET_DIRECTION: AtomicCell<Direction> = AtomicCell::new(Direction::Forward);
            static LIFT_SPEED: AtomicCell<u16> = AtomicCell::new(0);
            static BUCKET_SPEED: AtomicCell<u16> = AtomicCell::new(0);

            static LIFT_TARGET: AtomicCell<f64> = AtomicCell::new(0.0);
            static BUCKET_TARGET: AtomicCell<f64> = AtomicCell::new(0.0);

            if let Some((_, Some(actuator_cmd))) = input.payload() {
                match actuator_cmd {
                    embedded_common::ActuatorCommand::SetSpeed(speed, actuator) => match actuator {
                        embedded_common::Actuator::Lift => {
                            LIFT_SPEED.store(*speed);
                        }
                        embedded_common::Actuator::Bucket => {
                            BUCKET_SPEED.store(*speed);
                        }
                    },
                    embedded_common::ActuatorCommand::SetDirection(direction, actuator) => {
                        match actuator {
                            embedded_common::Actuator::Lift => {
                                LIFT_DIRECTION.store(*direction);
                            }
                            embedded_common::Actuator::Bucket => {
                                BUCKET_DIRECTION.store(*direction);
                            }
                        }
                    }
                    embedded_common::ActuatorCommand::Shake => {}
                    embedded_common::ActuatorCommand::StartPercuss => {}
                    embedded_common::ActuatorCommand::StopPercuss => {}
                }
            }

            let lift_speed = LIFT_SPEED.load();
            let bucket_speed = BUCKET_SPEED.load();

            let speed_scale = 0.01;

            let mut lift_target = LIFT_TARGET.load();
            if lift_speed > 0 {
                let speed_normalized = (lift_speed as f64) / 65535.0;
                let delta = speed_normalized * speed_scale;
                lift_target += match LIFT_DIRECTION.load() {
                    Direction::Forward => delta,
                    Direction::Backward => -delta,
                };
                lift_target = lift_target.clamp(-3.5, 1.0);
                LIFT_TARGET.store(lift_target);
            }
            data.actuator("lift_cylinder")
                .expect("lift_cylinder actuator not found")
                .view_mut(data)
                .ctrl[0] = lift_target;

            let mut bucket_target = BUCKET_TARGET.load();
            if bucket_speed > 0 {
                let speed_normalized = (bucket_speed as f64) / 65535.0;
                let delta = speed_normalized * speed_scale;
                bucket_target += match BUCKET_DIRECTION.load() {
                    Direction::Forward => delta,
                    Direction::Backward => -delta,
                };
                bucket_target = bucket_target.clamp(-2.0, 3.0);
                BUCKET_TARGET.store(bucket_target);
            }
            data.actuator("bucket_cylinder")
                .expect("bucket_cylinder actuator not found")
                .view_mut(data)
                .ctrl[0] = bucket_target;

            SimOverride::ExecutedBySim
        }
        default::SimStep::V3Pico(..) => SimOverride::ExecutedBySim,
        default::SimStep::MotorCtrl(CuTaskCallbackState::Process(input, _)) => {
            if let Some((Some(steering), _)) = input.payload() {
                let (left, right) = steering.get_left_and_right();
                let speed_mult = steering.get_weight();
                let left = (left * speed_mult) * 0.05;
                let right = (right * speed_mult) * 0.05;
                // left vesc
                data.actuator("motor_fl").unwrap().view_mut(data).ctrl[0] = left;
                data.actuator("motor_bl").unwrap().view_mut(data).ctrl[0] = left;
                // right vesc
                data.actuator("motor_fr").unwrap().view_mut(data).ctrl[0] = right;
                data.actuator("motor_br").unwrap().view_mut(data).ctrl[0] = right;
            }
            SimOverride::ExecutedBySim
        }
        default::SimStep::MotorCtrl(..) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::RealsenseSubscriber(CuTaskCallbackState::Process(_, output)) => {
            use std::sync::atomic::{AtomicU64, Ordering};
            use std::time::SystemTime;

            static LAST_OUTPUT_TIME: OnceLock<AtomicU64> = OnceLock::new();

            let now_ns = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            let output_interval_ns = 1_000_000_000 / 20; // output at 20 Hz
            let last_output_time = LAST_OUTPUT_TIME.get_or_init(|| AtomicU64::new(0));

            let last = last_output_time.load(Ordering::Relaxed);
            if now_ns - last > output_interval_ns {
                last_output_time.store(now_ns, Ordering::Relaxed);

                let depth_image = renderer.depth_flat();

                const DEPTH_SIZE: usize = DEPTH_FRAME_WIDTH as usize * DEPTH_FRAME_HEIGHT as usize;
                let mut depths = [0u16; DEPTH_SIZE];
                let depth_scale = 0.001; // 1 unit = 1mm

                if let Some(depth_image) = depth_image {
                    for (i, value) in depth_image.iter().enumerate() {
                        let depth_mm = (value * 1000.0).clamp(0.0, 65535.0) as u16;
                        // Mirror the image left to right
                        let row = i / DEPTH_FRAME_WIDTH as usize;
                        let col = i % DEPTH_FRAME_WIDTH as usize;
                        let mirrored_col = DEPTH_FRAME_WIDTH as usize - 1 - col;
                        let mirrored_idx = row * DEPTH_FRAME_WIDTH as usize + mirrored_col;
                        depths[mirrored_idx] = depth_mm;
                    }

                    let focal_len = (383.0, 383.0);

                    output.set_payload((
                        Some(IceoryxDepthFrame {
                            depths,
                            depth_scale,
                            focal_len,
                        }),
                        None,
                    ));
                }
            }
            SimOverride::ExecutedBySim
        }
        default::SimStep::RealsenseSubscriber(..) => SimOverride::ExecutedBySim,
        default::SimStep::T265Subscriber(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectionHandler(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2KissIcp(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::OccupancyGridPipeline(..) => SimOverride::ExecuteByRuntime,
        default::SimStep::NewAi(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::Localizer(CuTaskCallbackState::Process(_, output)) => {
            use std::sync::atomic::{AtomicU64, Ordering};
            use std::time::SystemTime;

            static LAST_LOG_TIME: OnceLock<AtomicU64> = OnceLock::new();

            let now_ns = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_nanos() as u64;

            let log_interval_ns = 1_000_000_000 / 60; // log at 60 Hz
            let last_log_time = LAST_LOG_TIME.get_or_init(|| AtomicU64::new(0));

            if let Some(state) = ROBOT_STATE.get()
                && let Some(lunabot_body) = data.body("simplify_lunabot")
                && let Some(velocimeter) = data.sensor("lunabot_velocimeter")
                && let Some(gyro) = data.sensor("lunabot_gyro")
            {
                let coords = lunabot_body.view(&data).xpos.to_vec();
                let quat = lunabot_body.view(&data).xquat.to_vec();
                if coords.len() != 3 || quat.len() != 4 {
                    eprintln!("Unexpected pose buffer len");
                    return SimOverride::ExecutedBySim;
                }
                // Offset from MuJoCo body origin to robot center (matches center of mass in simplify_lunabot.xml)
                // The kinematic chain in lunabot.ron assumes origin at robot center
                let body_origin_to_center = Translation3::new(0.37, -0.18, 0.00);
                let isometry = Isometry3::from_parts(
                    Translation3::from(
                        Translation3::new(coords[0], coords[1], coords[2]).vector
                            + body_origin_to_center.vector,
                    ),
                    UnitQuaternion::from_quaternion(Quaternion::new(
                        quat[0], quat[1], quat[2], quat[3],
                    )),
                );
                state.kinematic_root.set_isometry(isometry);
                output.set_payload(FromLunabot::RobotIsometry {
                    origin: isometry.translation.vector.cast::<f32>().data.0[0],
                    quat: isometry.rotation.as_vector().cast::<f32>().data.0[0],
                });

                // [x,y,z]
                let velocity = velocimeter.view(data).data.to_vec();

                let angular_velocity = gyro.view(data).data.to_vec();

                // store the state:
                // State: [x, y, z, vx, vy, vz, orientationerrorx, orientationerrory, orientationerrorz, wx, wy, wz]
                let kalman_state = SVector::<f64, 12>::from_row_slice(&[
                    coords[0],
                    coords[1],
                    coords[2],
                    velocity[0],
                    velocity[1],
                    velocity[2],
                    0.0,
                    0.0,
                    0.0, // orientation error starts at zero
                    angular_velocity[0],
                    angular_velocity[1],
                    angular_velocity[2],
                ]);
                // we are just going to ignore variances for now
                state.kalman_state.store(Some(kalman_state));

                let last = last_log_time.load(Ordering::Relaxed);
                if now_ns - last > log_interval_ns {
                    last_log_time.store(now_ns, Ordering::Relaxed);
                    if let Some(recorder) = RECORDER.get() {
                        let _ = recorder.recorder.log(
                            ROBOT_STRUCTURE,
                            &rerun::Transform3D::from_translation_rotation(
                                isometry.translation.vector.cast::<f32>().data.0[0],
                                rerun::Quaternion::from_xyzw(
                                    isometry.rotation.as_vector().cast::<f32>().data.0[0],
                                ),
                            ),
                        );
                        let cam_translation = data
                            .camera("front_depth_camera")
                            .unwrap()
                            .view(data)
                            .xpos
                            .to_vec();
                        let cam_mat = data
                            .camera("front_depth_camera")
                            .unwrap()
                            .view(data)
                            .xmat
                            .to_vec();

                        // Convert translation to [f32; 3]
                        let cam_pos = [
                            cam_translation[0] as f32,
                            cam_translation[1] as f32,
                            cam_translation[2] as f32,
                        ];

                        // Convert 3x3 rotation matrix to quaternion
                        let rot_mat = nalgebra::Matrix3::from_row_slice(&cam_mat);
                        let cam_quat = UnitQuaternion::from_matrix(&rot_mat);

                        let actual_depth_camera_pose = state
                            .kinematic_root
                            .get_node_with_name("upper_depth_camera")
                            .unwrap()
                            .get_global_isometry();

                        let _ = recorder.recorder.log(
                            "actual_depth_camera_pose",
                            &Transform3D::from_translation_rotation(
                                actual_depth_camera_pose
                                    .translation
                                    .vector
                                    .cast::<f32>()
                                    .data
                                    .0[0],
                                rerun::Quaternion::from_xyzw(
                                    actual_depth_camera_pose
                                        .rotation
                                        .as_vector()
                                        .cast::<f32>()
                                        .data
                                        .0[0],
                                ),
                            ),
                        );

                        let _ = recorder.recorder.log(
                            "depth_cam_pose",
                            &Transform3D::from_translation_rotation(
                                cam_pos,
                                rerun::Quaternion::from_xyzw([
                                    cam_quat.i as f32,
                                    cam_quat.j as f32,
                                    cam_quat.k as f32,
                                    cam_quat.w as f32,
                                ]),
                            ),
                        );
                    }
                }
            }
            SimOverride::ExecutedBySim
        }
        default::SimStep::Localizer(..) => SimOverride::ExecutedBySim,
        default::SimStep::LunabaseBridgeRxFromLunabaseRx(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::LunabaseBridgeTxToLunabase(_) => SimOverride::ExecuteByRuntime,

        default::SimStep::__Phantom(_) => SimOverride::ExecutedBySim,
    }
}

fn main() {
    if is_gpu_initialized() == false {
        init_gpu_blocking().expect("failed to init gpu");
    }

    // Setup logging path for resimulation
    let logger_path = "logs/lunabotsim.copper";
    if let Some(parent) = Path::new(logger_path).parent() {
        if !parent.exists() {
            std::fs::create_dir_all(parent).expect("Failed to create logs directory");
        }
    }
    // Create mock robot clock for simulation
    let (robot_clock, robot_clock_mock) = RobotClock::mock();
    let (model, mut viewer, data, mut renderer) = set_up_mujoco();

    let copper_ctx = basic_copper_setup(
        &PathBuf::from(&logger_path),
        PREALLOCATED_STORAGE_SIZE,
        true,
        Some(robot_clock.clone()),
    )
    .expect("Failed to setup logger.");

    rerun_viz::init_rerun(rerun_viz::RerunViz::Viz(rerun_viz::Level::All))
        .expect("Failed to initialize Rerun. Please check that the rerun binary is in your path.");

    let robot_chain = NodeSerde::from_reader(
        std::fs::File::open("../robot-layout/lunabot.ron").expect("Failed to read robot chain"),
    )
    .expect("Failed to parse robot chain");

    let robot_chain = ChainBuilder::from(robot_chain).finish_static();
    let _ = ROBOT_STATE.set(RobotState {
        kinematic_root: robot_chain,
        kalman_state: Arc::new(AtomicCell::new(Some(SVector::<f64, 12>::from_element(0.0)))),
        kalman_variances: Arc::new(AtomicCell::new(Some(
            SMatrix::<f64, 12, 12>::from_diagonal_element(1E64),
        ))),
    });
    let mut application = LunabotApplicationBuilder::new()
        .with_context(&copper_ctx)
        .with_sim_callback(&mut default_callback)
        .build()
        .expect("Failed to create application.");

    let mut sim_cb =
        |step: default::SimStep| -> SimOverride { sim_callback(step, data, &mut renderer) };

    application
        .start_all_tasks(&mut sim_cb)
        .expect("Failed to start all tasks.");

    if let Some(rec) = RECORDER.get() {
        let axes =
            rerun::Arrows3D::from_vectors([[0.5, 0.0, 0.0], [0.0, 0.5, 0.0], [0.0, 0.0, 0.5]])
                .with_colors([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
                .with_labels(vec!["x", "y", "z"]);

        let _ = rec.recorder.log("depth_cam_pose", &axes);
        let _ = rec.recorder.log("actual_depth_camera_pose", &axes);
    }

    let target_duration = std::time::Duration::from_nanos(1_000_000_000 / TARGET_HZ as u64);
    let mut last_time = std::time::Instant::now();
    let start = std::time::Instant::now();
    let mut counter = 0;
    while viewer.running() {
        counter += 1;
        robot_clock_mock.set_value(start.elapsed().as_nanos() as u64);
        {
            let mut sim_cb =
                |step: default::SimStep| -> SimOverride { sim_callback(step, data, &mut renderer) };

            application
                .run_one_iteration(&mut sim_cb)
                .expect("failed to run copper list iteration");
        }
        let elapsed = last_time.elapsed();
        data.step();
        if elapsed < target_duration {
            std::thread::sleep(target_duration - elapsed);
        }
        // we don't need to sync and render at TARGET_HZ, only step the simulation
        if counter == 20 {
            viewer.sync();
            renderer.sync(data);
            viewer.render(true);
            counter = 0;
        }
        last_time = std::time::Instant::now();
    }

    debug!("End of log replay.");
}

fn set_up_mujoco() -> (
    &'static MjModel,
    MjViewerCpp<&'static MjModel>,
    &'static mut MjData<&'static MjModel>,
    mujoco_rs::renderer::MjRenderer<&'static MjModel>,
) {
    println!("Creating model...");
    let args: Vec<String> = std::env::args().collect();
    let model = Box::leak(Box::new(if args.contains(&"artemis".to_string()) {
        MjModel::from_xml("../mujoco-sim/artemis_arena.xml")
            .expect("failed to create MjModel from artemis_arena.xml")
    } else if args.contains(&"ucf".to_string()) {
        MjModel::from_xml("../mujoco-sim/ucf_arena.xml")
            .expect("failed to create MjModel from ucf_arena.xml")
    } else {
        panic!(
            "Arena not specified in arguments. Valid args: ucf, artemis. (set with SIM_ARENA=ucf make sim)"
        );
    }));

    let timestep = 1.0 / (TARGET_HZ as f64);
    // speed up the simulation by a little
    // timestep *= 1.6;
    model.opt_mut().timestep = timestep;

    model.opt_mut().disableflags |= MjtDisableBit::mjDSBL_NATIVECCD as i32;
    model.opt_mut().enableflags |= MjtEnableBit::mjENBL_MULTICCD as i32;

    println!("Making Data...");
    let data = model.make_data();
    let data = Box::leak(Box::new(data));
    data.actuator("lift_cylinder")
        .expect("lift_cylinder actuator not found")
        .view_mut(data)
        .ctrl[0] = -3.0;
    println!("Launching Viewer...");

    let viewer = MjViewerCpp::launch_passive(model as &_, data as &_, 100);

    let camera = MjvCamera::new_fixed(
        data.camera("front_depth_camera")
            .expect("front_depth_camera not found")
            .id as u32,
    );
    let renderer = MjRendererBuilder::new()
        .depth(true)
        .num_visual_internal_geom(900)
        .camera(camera)
        .width(DEPTH_FRAME_WIDTH)
        .height(DEPTH_FRAME_HEIGHT)
        .build(model as &_)
        .expect("Failed to build renderer");
    (model, viewer, data, renderer)
}
