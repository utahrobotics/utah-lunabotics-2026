# Lunabot-CU: Lunar Excavation Robot Control System

A modular robotics framework for controlling a lunar excavation robot using the **Copper** real-time task framework. This system handles multi-camera vision processing, LIDAR point clouds, IMU data, and robot localization for autonomous lunar terrain navigation and excavation.

## Architecture Overview

The system is built on the [Copper framework](https://github.com/copper-project/copper-rs), which provides:
- **Real-time task execution** with deterministic scheduling
- **Message-passing communication** between tasks using typed channels  
- **Zero-copy data sharing** for high-performance sensor processing
- **Distributed processing** across multiple cores and nodes
- **Configuration-driven architecture** using RON (Rust Object Notation)

### Core Components

- **Vision System**: Multi-camera setup with AprilTag detection for localization
- **LIDAR Processing**: Unitree L2 LIDAR integration via iceoryx2 IPC
- **Robot State**: IMU-based orientation tracking and kinematic modeling
- **Localization**: Sensor fusion for robot pose estimation
- **Data Logging**: Real-time visualization and recording using Rerun

## Dependencies

### Required Dependencies

1. **iceoryx2** - Zero-copy inter-process communication framework
   ```bash
   # Ubuntu installation
   https://iceoryx2.readthedocs.io/getting_started.html#ubuntu
   # Build with examples off
   ```

2. **Copper Framework** - Real-time robotics task framework
   ```bash
   # Automatically pulled from git in Cargo.toml
   https://github.com/matthewashton-k/copper-rs.git
   ```

### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   ```bash
   https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild
   ```

## Configuration System

The system uses a hierarchical configuration approach with the **Copper** framework:

### Main Configuration (`copperconfig.ron`)

The primary configuration file defines:
- **Tasks**: Individual processing components with their types and configs
- **Connections**: Message flow between tasks using typed channels
- **Includes**: Template instantiation for reusable components
- **Logging**: System-wide logging and monitoring settings

```ron
(
    tasks: [
        (id: "udev_monitor", type: "crate::tasks::UdevMonitor", config: {}),
        (id: "l2_pointcloud", type: "crate::tasks::sources::PointCloudIceoryxReceiver", config: {}),
        // ... more tasks
    ],
    cnx: [
        (src: "l2_pointcloud", dst: "l2_pointcloud_sink", msg: "crate::tasks::PointCloudPayload"),
        // ... more connections
    ],
    includes: [
        // Template instantiations (see below)
    ],
    logging: (
        enable_task_logging: true,
        section_size_mib: 8,
    ),
)
```

### Template System with Includes

The configuration uses a powerful **template system** to instantiate multiple similar components:

#### Camera Template (`camera_configs/camera_template_gstreamer.ron`)

This template defines a complete camera processing pipeline that gets instantiated for each physical camera:

```ron
(
    tasks: [
        (id: "cam_{{id}}", type: "crate::tasks::CuDefaultAutoGStreamer", config: {
            "camera_id": "cam_{{id}}",
            "device_port": "{{port}}",
            "pipeline": "v4l2src device=<devpath> ! video/x-raw,width={{width}},height={{height}} ! ...",
            // GStreamer pipeline with parameter substitution
        }),
        (id: "thres_{{id}}", type: "cu_dynthreshold::DynThreshold", config: {
            "width": {{width}}, "height": {{height}}, "block_radius": 100
        }),
        (id: "detector_cam_{{id}}", type: "cu_apriltag::AprilTags", config: {
            "fx": {{fx}}, "fy": {{fy}}, "cx": {{cx}}, "cy": {{cy}}, // Camera intrinsics
        }),
        // Shared tasks (detection_handler, localizer)
    ],
    cnx: [
        (src: "cam_{{id}}", dst: "thres_{{id}}", msg: "crate::tasks::CuGstBuffer"),
        (src: "thres_{{id}}", dst: "detector_cam_{{id}}", msg: "cu_sensor_payloads::CuImage<Vec<u8>>"),
        // ... complete processing chain
    ],
)
```

#### Template Instantiation

Templates are instantiated in the main config with specific parameters:

```ron
includes: [
    (
        path: "camera_configs/camera_template_gstreamer.ron",
        params: {
            "id": "back",                           // Camera identifier
            "port": "pci-0000:35:00.0-usb-0:1:1.0", // USB device port
            "fx": 689.93, "fy": 689.93,             // Camera intrinsics
            "cx": 320.84, "cy": 240.819,
            "width": 640, "height": 480,
            "udp_host": "127.0.0.1", "udp_port": 5004, // Video streaming
        },
    ),
    // Additional camera instances...
]
```

This creates a complete camera processing pipeline for each camera with custom parameters while sharing common processing tasks.

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

## Task Architecture

The system follows a **dataflow architecture** where tasks communicate via typed message channels:

### Task Categories

#### Source Tasks (`src/tasks/sources/`)
- **UdevMonitor**: Detects camera connection/disconnection events
- **PointCloudIceoryxReceiver**: Receives LIDAR data via iceoryx2 IPC  
- **ImuIceoryxReceiver**: Receives IMU data from the LIDAR unit
- **Teleop**: TODO Manual robot control interface

#### Processing Tasks (`src/tasks/`)
- **CuDefaultAutoGStreamer**: Camera capture and video streaming
- **DynThreshold**: Adaptive image thresholding for AprilTag detection
- **AprilTags**: Fiducial marker detection for localization
- **AprilDetectionHandler**: Aggregates detections from multiple cameras

#### Sink Tasks (`src/tasks/sinks/`)
- **L2PointCloudSink**: LIDAR data logging and visualization
- **CuLocalizer**: Robot pose estimation and state tracking

### Data Flow Example

```
UdevMonitor → AutoGStreamer → DynThreshold → AprilTags → DetectionHandler → Localizer
                    ↓
              UDP Video Stream

L2 LIDAR → PointCloudReceiver → PointCloudSink → Rerun Visualization
         → ImuReceiver → Localizer → Robot State Updates
```

## System Operation

### Camera Processing Pipeline

1. **UdevMonitor** detects when cameras are plugged in
2. **AutoGStreamer** matches device ports to configured cameras and starts capture
3. **DynThreshold** applies adaptive thresholding for marker detection
4. **AprilTags** detects fiducial markers and estimates camera poses
5. **DetectionHandler** aggregates multi-camera observations
6. **Localizer** updates robot pose using camera-based localization

### LIDAR Processing Pipeline

1. **unilidar_iceoryx_publisher** (C++) captures L2 LIDAR data and publishes via iceoryx2
2. **PointCloudReceiver** consumes point clouds and transforms to robot coordinates
3. **ImuReceiver** processes inertial data for orientation tracking
4. **PointCloudSink** logs data to Rerun for visualization
5. **Localizer** incorporates IMU data for robot orientation updates

### Coordinate System Integration

- **L2 LIDAR coordinates** → **Robot base frame** via kinematic transforms
- **Camera coordinates** → **Robot base frame** via AprilTag observations
- **Robot base frame** → **Global frame** via sensor fusion in localizer

## Building and Running

```bash
# build and run unilidar publisher 
mkdir build && cd build
cmake ..
make
cd ../
./run_publisher.sh

# build and run the lunabot
make prod # no logging
made debug # logging
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
- **Message Flow**: Live message passing between tasks  
- **Performance Metrics**: CPU usage, memory, and timing information
- **System Health**: Error rates and task failure detection
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