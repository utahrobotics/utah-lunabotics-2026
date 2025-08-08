# Lunabot

A modular robotics framework for controlling a lunar excavation robot using the **Copper** real-time task framework. This system handles multi-camera vision processing, LIDAR point clouds, IMU data, and robot localization for autonomous lunar terrain navigation and excavation.

*This repo is in the early stages of a rewrite migrating to the copper framework from last years code base: github.com/utahrobotics/lunadev-2025*

## Architecture Overview

The system is built on the [Copper framework](https://github.com/copper-project/copper-rs), which provides:
- **Real-time task execution** with deterministic scheduling
- **Distributed processing** across multiple cores and nodes
- **Configuration-driven architecture** using RON (Rust Object Notation)
- **Inter-Process communication** using Iceoryx2 for integration with separate processes running in ROS2 or elsewhere

### Core Components

- **Vision System**: Multi-camera setup with AprilTag detection for localization
- **LIDAR Processing**: Unitree L2 and RealSense LIDAR integration via iceoryx2 IPC.
- **Robot State**: Rigid kinematic chain modeling.
- **Localization**: Sensor fusion from IMUs, KISS-ICP, and apriltags for robot pose estimation
- **Data Logging**: Real-time visualization and recording using Rerun
- **Teleop**: Using UDP for communication between the base and bot with a custom quality of service state machine.
- **Lunabase**: Base station software for controlling the robot as well as receiving telemetry and camera feeds.

## Dependencies

### Required Dependencies

1. Bazel 6.2 (via bazelisk)

2. LAPACK/openblas libraries:

```bash
sudo apt update && sudo apt install -y liblapack-dev libblas-dev libopenblas-dev
```

3. realsense2 SDK

4. GStreamer

```bash
apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly \
      gstreamer1.0-libav libgstrtspserver-1.0-dev libges-1.0-dev
```

5. libssl-dev

6. apriltag library https://github.com/AprilRobotics/apriltag?tab=readme-ov-file#install


### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   ```bash
   https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild
   ```
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


## System Operation

### Camera Processing Pipeline

1. **UdevMonitor** detects when cameras are plugged in
2. **AutoGStreamer** matches device ports to configured cameras and starts capture
3. **DynThreshold (or GstToImage)** applies adaptive thresholding for marker detection
4. **AprilTags** detects fiducial markers and estimates camera poses
5. **DetectionHandler** aggregates multi-camera observations
6. **Localizer** updates robot pose using camera-based localization

### LIDAR Processing Pipeline

1. **unilidar_iceoryx_publisher** (C++) captures L2 LIDAR data and publishes via iceoryx2
2. **PointCloudReceiver** consumes point clouds and transforms to robot coordinates
3. **ImuReceiver** processes inertial data for orientation tracking
5. **Localizer** incorporates IMU data for robot orientation updates

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

- **Copper logs**: Stored in `logs/lunabot.copper` for system debugging (only active on debug builds)
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


## Configuring
TODO
