# Lunabot

*This repo is in the earlyish stages of a rewrite migrating to the copper framework from last years code base: github.com/utahrobotics/lunadev-2025*

## Architecture Overview
<img width="4150" height="818" alt="graphviz(8)" src="https://github.com/user-attachments/assets/a53d91de-1872-44cc-9cf0-02a6a0bba5c2" />


##### Check copperconfig.ron to see the definitions of all the tasks running and the datatypes passed between tasks.

## Dependencies

### Production env (Linux only)
1. See dockerfile, anything installed there is a dependency for running the robot in production.
2. Apriltag C library and librealsense

### Log Replay (Native)

1. ```make``` command 
2. git
3. rerun - download binary from their github releases page and add "rerun" command to Path.
4. clang - for windows install with ```choco install llvm```, linux use ```apt install libclang-dev``` 
5. rust - install from rustup.rs (select nightly version)

NOTE: If you are on macos you can install git, libclang, and make by running ```xcode-select --install```

### Log Replay (Docker)
1. git
2. rerun 
3. docker

### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   ```bash
   https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild
   ```
2. iox2 cli tool for seeing active iceoryx2 nodes and services.
3. cargo flamegraph + perf for profiling
4. gdb
5. lz4 for compressing logs


## Building and Running
First install the dependencies listed in the dependencies section. 


*NOTE: On some machines you have to increase the stack size for it to compile ```export RUST_MIN_STACK=107108864```.*

### Production env
1. run ```make sync``` to build/sync deps for the Unitree L2 publisher.
2. run ```make prod``` to build and run the project.

### Log replay (Native)
1. Ensure there are valid log files in lunabot-cu/logs
2. run ```make resim```

### Log replay and development tools (MacOS)

#### Dependencies for log replay

1. Git

   This is pre installed on macOS. Just set it up with either a personal access token or SSH keys.

2. `make`

   Run `xcode-select --install` if you haven't before. This will install `make` along with many other useful development tools.

3. Rerun

   Download the latest macOS binary from releases section on <https://github.com/rerun-io/rerun>. The latest version is 0.24.1 at the time of writing. `rerun-cli-0.24.1-x86_64-apple-darwin` if you are on an Intel Mac, or `rerun-cli-0.24.1-aarch64-apple-darwin` if you are on Apple Silicon. Make it executable with `chmod +x`. Rename the file to `rerun` and move it to `~/.local/bin`. Assuming you are using zsh, `echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc` and `source ~/.zshrc` to add it to your PATH. `which rerun` to ensure it is recognized.

4. clang

   This is pre installed on macOS.

5. Rust

   Install from [rustup.rs.](rustup.rs). After installation, switch to the nightly release using `rustup default nightly`.

#### Other development tools

Recommended IDE : [VS Code](https://code.visualstudio.com)

Brew - This is a useful package manager for macOS. It will be helpful. Install from [brew.sh](https://brew.sh)

#### Running

   1. Clone the repository.
   2. Run `rerun` in your terminal to launch the GUI. If it is the first time running, you may need to go into privacy settings and approve it, since it is an unsigned app.
   3. In a new terminal window, in the repo directory, run `make resim`. It will take a few minutes to build and then launch the simulation in Rerun's GUI.


### Log replay (Docker)
1. Ensure there are valid log files in lunabot-cu/logs
2. From the repository root first build the docker container with ```docker build -t lunabot .``` then if using bash/gitbash run with ```docker run --network=host -it -v $(pwd):/workspace -v cargo-cache:/workspace/target -v cargo-registry:/usr/local/cargo/registry --name lunabot lunabot``` otherwise run with ```docker run --network=host -it -v "${PWD}:/workspace" -v cargo-cache:/workspace/target -v cargo-registry:/usr/local/cargo/registry --name lunabot lunabot```
3. Comment out the rerun init function call in lunabot-cu/src/resim.rs, and uncomment the line above with the Grpc config.
4. Start rerun on host.
5. Run replay with ```container$ make resim```

### Help
```bash
make help # see commands for building and running
```

## Camera discovery

- monitors udev events and allows for easy discovery of which cameras are on which ports.

```bash
make discover-cameras
```


# Crate Layout

## Entry points
### lunabot-cu/src/main.rs
* Launches exernal processes.
* Sets up rerun
* Serializes the robot chain. 
* Builds and runs the lunabot application.

### lunabot-cu/src/resim.rs
* Reads in copperlist logs from lunabot-cu/logs. These logs contain messages passed between copper tasks.
* Runs the lunabot in simulation mode which allows you to selectively decide which task's process functions are simulated "Mocked", and which tasks process functions are not.
* Allows you to set a task with a mocked process function to output to whatever was read in from the logs, effectively allowing for deterministic replay of whatever was captured.

### external-tasks/realsense
* Launched by lunabot-cu. 
* Detects when a realsense device is plugged in, automatically opens device and publishes point clouds.
* Not compiled or launched in log replay mode.

### unilidar_iceoryx_publisher
* Launched by lunabot-cu
* Connects to L2 and publishes imu data and pointclouds from it.
* Not compiled or executed in log replay mode.

## misc/ 
Contains libraries for kinematics, network protocols, GPU utilites/Shader pipelines, camera auto discovery, behavior trees, and interaction with VESC boards.

## lunabot-cu/src/tasks/sources
Contains source tasks for:

* Receiving messages from sensors in external tasks (Realsense, L2 lidar)
* Handling the connection to the lunabase.

## lunabot-cu/src/tasks/sinks
* Sink tasks for interacting with the actuators and motors
* localizer
* logger placeholder task for testing obstacle map generation

## lunabot-cu/src/tasks/ai
* Takes in readings from the lunabase. 
* Keeps track of what the robot is currently doing (see LunabotAction enum)
* Keeps track of state (the blackboard) associated with the lunabot: robot chain, last messages seen from lunabase, latest obstacle map, (and more to come)
* Decides how to control the motors and actuators based on all that information.

## lunabot-cu/src/tasks (not the source or sinks ones)
Tasks that lie between the sources and sinks for:
* Image processing for apriltag detection.
* Automatically opening camera devices as they become available.
* Point cloud processing (KISS-ICP)

## lunabot-cu/src/utils
Helper functions for:
* Framed codec for talking with rp2040
* Udev polling
* Linear interpolation
* Converting between units/types.

## lunabot-cu/src/comms
* Structures and helpers used for connecting to the base station.

## common/ 
Legacy code containing structures that are used by the lunabase and the lunabot. (and the old behavior tree). \\
*This will be useful when we actually move the lunabase to this repo*

## embedded_common
A no_std crate containing structures used by the embedded code as well as our tasks. 
*there is no embedded code in this repo yet because we don't know what electrical team will throw our way quite yet*

## Others

### lunabot-cu/src/rerun_viz.rs
Utilities for connecting to rerun.

### lunabot-cu/src/rp2040.rs
Legacy code for connecting to a rpi pico that is in charge of actuator control and IMUs. \\ 

The enumerate_v3picos() starts a thread that is in charge of communicating with the pico, and returns a Tx Rx pair such that you can:
* Command the actuators by sending messages to tx
* Read messages from the pico by recv from rx.
The tx side is used in the actuator_ctrl task.

### lunabot-cu/src/motors.rs
Legacy code for controlling motors via VESC. \\
The enumerate_motors() function (used by the motor_ctrl task) returns a structure that you can use to command the motors.

### lunabot-cu/src/simple_monitor.rs
Hooks into the copper runtime and prints messages when a task's process, preprocess, etc return Err.

Example output:
```
=== ERRORED TASKS ===
Task 10: lunabase (State: Process) - lunabase not connected
   context:lunabase disconnected
Task 17: cam_side (State: Process) - no frames received
   context:no frames received
Task 14: cam_back (State: Process) - no frames received
   context:no frames received
Task 4: realsense_pointcloud (State: Process) - No points seen in 600 ms
   context:No points seen in 600 ms
Task 5: realsense_occupancy (State: Process) - No occupancy grid seen in 600 ms
   context:No occupancy grid seen in 600 ms
Task 2: l2_pointcloud (State: Process) - No points seen in 600 ms
   context:No points seen in 600 ms
=====================
```
