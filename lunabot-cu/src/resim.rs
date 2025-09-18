#![feature(try_blocks, f16, mpmc_channel)]
pub mod comms;
pub mod rerun_viz;

pub mod simple_monitor;
pub mod tasks;
pub mod utils;

use crossbeam_channel::{Receiver, Sender};
use cu29::prelude::*;
use cu29_export::copperlists_reader;
use cu29_helpers::basic_copper_setup;
use embedded_common::{ActuatorCommand, FromPicoV3};
use simple_motion::{ChainBuilder, NodeSerde, StaticNode};
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

const PREALLOCATED_STORAGE_SIZE: Option<usize> = Some(1024 * 1024 * 100);

pub static ROOT_NODE: OnceLock<StaticNode> = OnceLock::new();

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
        default::SimStep::RealsenseOccupancy(_) => SimOverride::ExecutedBySim,
        default::SimStep::Lunabase(_) => SimOverride::ExecutedBySim,
        // May want to temporarily add override for obstacle map recv until lidar simulation works
        _ => SimOverride::ExecuteByRuntime,
    }
}

fn run_one_copperlist(
    copper_app: &mut LunabotApplication,
    robot_clock: &mut RobotClockMock,
    copper_list: CopperList<default::CuStampedDataSet>,
) {
    let msgs = &copper_list.msgs;
    let now = msgs
        .get_lunabase_output()
        .metadata()
        .process_time()
        .start
        .unwrap()
        .as_nanos();
    robot_clock.set_value(now);

    let mut sim_callback = move |step: default::SimStep| -> SimOverride {
        match step {
            default::SimStep::UdevMonitor(CuTaskCallbackState::Process(_, output)) => {
                output.tov = robot_clock.now().into();
                *output = msgs.get_udev_monitor_output().clone();
                SimOverride::ExecutedBySim
            }
            default::SimStep::UdevMonitor(..) => SimOverride::ExecutedBySim,

            default::SimStep::CamSide(..) => SimOverride::ExecutedBySim,
            default::SimStep::CamBack(..) => SimOverride::ExecutedBySim,

            default::SimStep::CamLaptopFront(..) => SimOverride::ExecutedBySim,
            default::SimStep::GstConvertBack(..) => SimOverride::ExecutedBySim,
            default::SimStep::GstConvertSide(..) => SimOverride::ExecutedBySim,
            default::SimStep::GstConvertLaptopFront(..) => SimOverride::ExecutedBySim,

            default::SimStep::L2Pointcloud(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_l_2_pointcloud_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::L2Pointcloud(..) => SimOverride::ExecutedBySim,

            default::SimStep::L2Imu(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_l_2_imu_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::L2Imu(..) => SimOverride::ExecutedBySim,

            default::SimStep::V3Pico(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_v_3_pico_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::V3Pico(..) => SimOverride::ExecutedBySim,

            default::SimStep::MotorCtrl(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_motor_ctrl_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::MotorCtrl(..) => SimOverride::ExecutedBySim,
            default::SimStep::DetectorCamBack(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_detector_cam_back_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::DetectorCamBack(..) => SimOverride::ExecutedBySim,
            default::SimStep::DetectorCamSide(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_detector_cam_side_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::DetectorCamSide(..) => SimOverride::ExecutedBySim,
            default::SimStep::DetectorCamLaptopFront(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_detector_cam_laptop_front_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::RealsenseOccupancy(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_realsense_occupancy_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::RealsenseOccupancy(..) => SimOverride::ExecutedBySim,
            default::SimStep::Lunabase(CuTaskCallbackState::Process(_, output)) => {
                *output = msgs.get_lunabase_output().clone();
                output.tov = robot_clock.now().into();
                SimOverride::ExecutedBySim
            }
            default::SimStep::Lunabase(..) => SimOverride::ExecutedBySim,
            default::SimStep::DetectorCamLaptopFront(..) => SimOverride::ExecutedBySim,

            // May want to temporarily add override for obstacle map recv until lidar simulation works
            _ => SimOverride::ExecuteByRuntime,
        }
    };

    copper_app
        .run_one_iteration(&mut sim_callback)
        .expect("Failed to run application iteration.");
}

fn main() {
    std::thread::Builder::new()
        .stack_size(20 * 1000000) // this size will depend on how big the copper list is
        .spawn(move || {
            // Setup logging path for resimulation
            let logger_path = "logs/lunabotsim.copper";
            if let Some(parent) = Path::new(logger_path).parent() {
                if !parent.exists() {
                    std::fs::create_dir_all(parent).expect("Failed to create logs directory");
                }
            }
            // Create mock robot clock for simulation
            let (robot_clock, mut robot_clock_mock) = RobotClock::mock();

            let copper_ctx = basic_copper_setup(
                &PathBuf::from(&logger_path),
                PREALLOCATED_STORAGE_SIZE,
                true,
                Some(robot_clock.clone()),
            )
            .expect("Failed to setup logger.");

            // uncomment to use with docker
            // rerun_viz::init_rerun(rerun_viz::RerunViz::Grpc(
            //     rerun_viz::Level::All,
            //     "host.docker.internal".to_string(),
            // ))
            // .expect("Failed to initialize rerun viz.");

            let _ = rerun_viz::init_rerun(rerun_viz::RerunViz::Viz(rerun_viz::Level::All));

            let robot_chain = NodeSerde::from_reader(
                std::fs::File::open("../robot-layout/lunabot.ron")
                    .expect("Failed to read robot chain"),
            )
            .expect("Failed to parse robot chain");

            let robot_chain = ChainBuilder::from(robot_chain).finish_static();
            let _ = ROOT_NODE.set(robot_chain);

            let mut application = LunabotApplicationBuilder::new()
                .with_sim_callback(&mut default_callback)
                .with_context(&copper_ctx)
                .build()
                .expect("Failed to create application.");

            application
                .start_all_tasks(&mut default_callback)
                .expect("Failed to start all tasks.");

            let UnifiedLogger::Read(dl) = UnifiedLoggerBuilder::new()
                .file_base_name(Path::new("logs/lunabot.copper"))
                .build()
                .expect("Failed to create logger")
            else {
                panic!("Failed to create logger");
            };

            let mut reader = UnifiedLoggerIOReader::new(dl, UnifiedLogType::CopperList);
            let copperlists = copperlists_reader::<default::CuStampedDataSet>(&mut reader);

            let target_duration = std::time::Duration::from_nanos(1_000_000_000 / TARGET_HZ as u64);
            let mut last_time = std::time::Instant::now();

            for copper_list in copperlists {
                run_one_copperlist(&mut application, &mut robot_clock_mock, copper_list);

                let elapsed = last_time.elapsed();
                if elapsed < target_duration {
                    std::thread::sleep(target_duration - elapsed);
                }
                last_time = std::time::Instant::now();
            }

            application
                .stop_all_tasks(&mut default_callback)
                .expect("Failed to stop all tasks.");
        })
        .expect("failed to spawn main thread")
        .join()
        .expect(".join() on main thread failed");

    debug!("End of log replay.");
}
