#![feature(try_blocks, f16, mpmc_channel)]
pub mod comms;
pub mod rerun_viz;

pub mod bridges;
pub mod pathfinding;
pub mod simple_monitor;
pub mod tasks;
pub mod utils;

use common::FromLunabot;
use cu29::prelude::*;
use cu29_helpers::basic_copper_setup;
use kalman_filter::SimpleSquareMatrix;
use kalman_filter::SimpleVector;
use mujoco_rs::cpp_viewer::MjViewerCpp;
use mujoco_rs::prelude::*;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::{Mutex, OnceLock, RwLock};
use utils::RobotState;
use wgsl_pcl::wgsl_setup::{init_gpu_blocking, is_gpu_initialized};

use crate::rerun_viz::{RECORDER, ROBOT_STRUCTURE};

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

pub static ROBOT_STATE: OnceLock<RobotState> = OnceLock::new();

pub static TARGET_HZ: usize = 1000; // MUST BE THE SAME AS THE TARGET HZ IN COPPERCONFIG.RON

pub static MJ_MODEL: OnceLock<&'static MjModel> = OnceLock::new();
pub static MJ_DATA: OnceLock<Mutex<&'static mut MjData<&'static MjModel>>> = OnceLock::new();

#[copper_runtime(config = "copperconfig.ron", sim_mode = true)]
struct LunabotApplication {}

fn sim_callback(step: default::SimStep) -> SimOverride {
    // mj model and data should always be set before this function is called
    let model = MJ_MODEL.get().unwrap();
    let mut data = MJ_DATA.get().unwrap().lock().unwrap();
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
        default::SimStep::MotorCtrl(CuTaskCallbackState::Process(input, _)) => {
            if let Some((Some(steering), _)) = input.payload() {
                let (left, right) = steering.get_left_and_right();
                let speed_mult = steering.get_weight();
                let left = (left * speed_mult) * 0.05;
                let right = (right * speed_mult) * 0.05;
                // left vesc
                data.actuator("motor_fl").unwrap().view_mut(&mut data).ctrl[0] = left;
                data.actuator("motor_bl").unwrap().view_mut(&mut data).ctrl[0] = left;
                // right vesc
                data.actuator("motor_fr").unwrap().view_mut(&mut data).ctrl[0] = right;
                data.actuator("motor_br").unwrap().view_mut(&mut data).ctrl[0] = right;
            }
            SimOverride::ExecutedBySim
        }
        default::SimStep::MotorCtrl(..) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamBack(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamSide(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectorCamLaptopFront(_) => SimOverride::ExecutedBySim,
        default::SimStep::RealsenseSubscriber(_) => SimOverride::ExecutedBySim,
        default::SimStep::T265Subscriber(_) => SimOverride::ExecutedBySim,
        default::SimStep::DetectionHandler(_) => SimOverride::ExecutedBySim,
        default::SimStep::L2KissIcp(_) => SimOverride::ExecuteByRuntime,
        default::SimStep::OccupancyGridPipeline(_) => SimOverride::ExecuteByRuntime,
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
            {
                let coords = lunabot_body.view(&data).xpos.to_vec();
                let quat = lunabot_body.view(&data).xquat.to_vec();
                if coords.len() != 3 || quat.len() != 4 {
                    eprintln!("Unexpected pose buffer len");
                    return SimOverride::ExecutedBySim;
                }
                let isometry = Isometry3::from_parts(
                    Translation3::new(coords[0], coords[1], coords[2]),
                    UnitQuaternion::from_quaternion(Quaternion::new(
                        quat[0], quat[2], quat[2], quat[3],
                    )),
                );
                state.kinematic_root.set_isometry(isometry);
                output.set_payload(FromLunabot::RobotIsometry {
                    origin: isometry.translation.vector.cast::<f32>().data.0[0],
                    quat: isometry.rotation.as_vector().cast::<f32>().data.0[0],
                });

                let last = last_log_time.load(Ordering::Relaxed);
                if now_ns - last > log_interval_ns {
                    last_log_time.store(now_ns, Ordering::Relaxed);
                    if let Some(recorder) = RECORDER.get()
                        && let Err(e) = recorder.recorder.log(
                            rerun_viz::ROBOT_STRUCTURE,
                            &rerun::Transform3D::from_translation_rotation(
                                isometry.translation.vector.cast::<f32>().data.0[0],
                                rerun::Quaternion::from_xyzw(
                                    isometry.rotation.as_vector().cast::<f32>().data.0[0],
                                ),
                            ),
                        )
                    {
                        eprintln!("Failed to log robot pose: {}", e);
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
    std::thread::Builder::new()
        .stack_size(20 * 1000000) // this size will depend on how big the copper list is
        .spawn(move || {
            let (model, mut viewer, data) = set_up_mujoco();
            MJ_MODEL.set(model).expect("Failed to set mj model");

            if MJ_DATA.set(Mutex::new(data)).is_err() {
                panic!("Failed to set mj data");
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

            let copper_ctx = basic_copper_setup(
                &PathBuf::from(&logger_path),
                PREALLOCATED_STORAGE_SIZE,
                true,
                Some(robot_clock.clone()),
            )
            .expect("Failed to setup logger.");

          //  rerun_viz::init_rerun(rerun_viz::RerunViz::Viz(rerun_viz::Level::All)).expect(
            ///    "Failed to initialize Rerun. Please check that the rerun binary is in your path.",
            //);

            let robot_chain = NodeSerde::from_reader(
                std::fs::File::open("../robot-layout/lunabot.ron")
                    .expect("Failed to read robot chain"),
            )
            .expect("Failed to parse robot chain");

            let robot_chain = ChainBuilder::from(robot_chain).finish_static();
            let _ = ROBOT_STATE.set(RobotState {
                kinematic_root: robot_chain,
                kalman_state: Arc::new(RwLock::new(SimpleVector::<15>::from_element(0.0))),
                kalman_variances: Arc::new(RwLock::new(
                    SimpleSquareMatrix::<15>::from_diagonal_element(1E64),
                )),
            });

            let mut application = LunabotApplicationBuilder::new()
                .with_sim_callback(&mut sim_callback)
                .with_context(&copper_ctx)
                .build()
                .expect("Failed to create application.");

            application
                .start_all_tasks(&mut sim_callback)
                .expect("Failed to start all tasks.");
            let target_duration = std::time::Duration::from_nanos(1_000_000_000 / TARGET_HZ as u64);
            let mut last_time = std::time::Instant::now();
            let start = std::time::Instant::now();
            let mut counter = 0;
            while viewer.running() {
                counter += 1;
                robot_clock_mock.set_value(start.elapsed().as_nanos() as u64);
                application
                    .run_one_iteration(&mut sim_callback)
                    .expect("failed to run copper list iteration");
                MJ_DATA.get().unwrap().lock().unwrap().step();

                let elapsed = last_time.elapsed();
                if elapsed < target_duration {
                    std::thread::sleep(target_duration - elapsed);
                }
                // we don't need to sync and render at TARGET_HZ, only step the simulation
                if counter == 20 {
                    viewer.sync();
                    viewer.render(true);
                    counter = 0;
                }
                last_time = std::time::Instant::now();
            }
        })
        .expect("failed to spawn main thread")
        .join()
        .unwrap_or_else(|e| {
            if let Some(panic_msg) = e.downcast_ref::<&str>() {
                panic!("Thread panicked with message: {}", panic_msg);
            } else if let Some(panic_msg) = e.downcast_ref::<String>() {
                panic!("Thread panicked with message: {}", panic_msg);
            } else {
                panic!("Thread panicked with unknown error: {:?}", e);
            }
        });

    debug!("End of log replay.");
}

fn set_up_mujoco() -> (
    &'static MjModel,
    MjViewerCpp<&'static MjModel>,
    &'static mut MjData<&'static MjModel>,
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
    let mut timestep = 1.0 / (TARGET_HZ as f64);
    // speed up the simulation by a little
    // timestep *= 1.6;
    model.opt_mut().timestep = timestep;

    model.opt_mut().disableflags |= MjtDisableBit::mjDSBL_NATIVECCD as i32;
    model.opt_mut().enableflags |= MjtEnableBit::mjENBL_MULTICCD as i32;
    println!("Making Data...");
    let data = model.make_data();
    let leaked_data = Box::leak(Box::new(data));
    println!("Launching Viewer...");

    let viewer = MjViewerCpp::launch_passive(model as &_, leaked_data as &_, 100);
    (model, viewer, leaked_data)
}
