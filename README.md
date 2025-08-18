# Lunabot

A modular robotics framework for controlling a lunar excavation robot using the **Copper** real-time task framework. This system handles multi-camera vision processing, LIDAR point clouds, IMU data, and robot localization for autonomous lunar terrain navigation and excavation.

*This repo is in the early stages of a rewrite migrating to the copper framework from last years code base: github.com/utahrobotics/lunadev-2025*

## Architecture Overview
<img width="4210" height="834" alt="graphviz" src="https://github.com/user-attachments/assets/7a088db9-2914-4817-95a1-f575d1092c70" />


## Dependencies

### Required Dependencies

1. See dockerfile, anything installed there is a dependency.


### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   ```bash
   https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild
   ```

2. Rerun - visualize data produced by the robot.


### Robot Kinematic Configuration (`robot-layout/lunabot.ron`)

Defines the robot's physical structure and sensor placements:

```ron
{
    free_origin: [0.0, 0.0, 0.0],
    free_euler: [0.0, 0.0, 180.0],
    children: [
        { name: Some("cam_back"), origin: [0.0, 0.0, 0.475], euler: [0.0, 0.0, 180.0] },
        { name: Some("l2_front"), origin: [0.0, 0.0, -0.475], euler: [0.0, -90.0, 0.0] },
        // ... complete kinematic tree
    ]
}
```


## Building and Running

```bash
make help # see commands for building and running
```

## Camera discovery

- monitors udev events and allows for easy discovery of which cameras are on which ports.

```bash
make discover-cameras
```

## Logging and Visualization

- **Copper logs**: Stored in `logs/lunabot.copper` for system debugging (messages only active on debug builds, full copperlist logging enabled when enable_task_logging is true)
- **Rerun visualization**: Real-time 3D visualization of robot state, point clouds, and camera feeds
- **Video streams**: Live UDP streams from cameras for remote monitoring

### Console Monitor (Task latency and health viewer)

The system includes an optional **terminal-based monitoring interface** using the Cursive TUI library for real-time system debugging and task monitoring.

#### What the Console Monitor Provides

The console monitor displays a live text-based interface showing:
- **Task Status**: Real-time status of all running tasks
- **System Health**: Error and task failure detection
- **Resource Usage**: Memory allocation and processing bottlenecks

#### Enabling the Console Monitor

To enable it, uncomment the monitor line in `copperconfig.ron`:

```ron
(
    // ... existing config ...
    monitor: (type: "cu_cursive_consolemon::CuCursiveConsoleMon"),
    // ... rest of config ...
)
```


## Get Started
_First either use docker or install dependencies listed in the dockerfile system wide_
1. clone repo
2. run `make sync` to sync the dependencies for the c++ portion of the codebase.
3. run `make prod` to build and run the robot.

If you have rerun installed, you should see a window pop up that begins to recieve data from the robot, and as the robot reads sensor data in it will update its isometry which you can watch live through rerun.

 TODO: lunabase instructions

## TODO: everything logging related
1. copper log macros only print in debug builds (I likely will change this)
2. console monitor stuff
3. rerun stuff

