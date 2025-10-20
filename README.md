# Lunabot

## Architecture Overview

<img width="4378" height="726" alt="graphviz" src="https://github.com/user-attachments/assets/399a1f5c-2bb3-446b-83d5-b68da5d1582f" />

##### Check copperconfig.ron to see the definitions of all the tasks running and the datatypes passed between tasks.

## Dependencies

### Production env (Linux only)

1. ```
   apt/dnf/yum/etc install \
    pkg-config \
    libssl-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libacl1-dev \
    libgstrtspserver-1.0-dev \
    libges-1.0-dev \
    libv4l-dev \
    libunwind-dev \
    libudev-dev \
   ```
2. [Bazelisk](http://github.com/bazelbuild/bazelisk/releases/)
3. [Realsense SDK](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#building-librealsense2-sdk)
4. [Apriltag Library](https://github.com/AprilRobotics/apriltag)

### Log Replay env (Windows, Macos, Linux)
See ```Building and Running``` section for detailed instructions for your platform.

### Simulation Environment 
See [mujoco-sim/README.md](mujoco-sim/README.md) for dependencies and instructions.

### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   <https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild>
2. iox2 cli tool for seeing active iceoryx2 nodes and services.
3. cargo flamegraph + perf for profiling
4. gdb
5. lz4 for compressing logs


## Building and Running

*NOTE: On some machines you have to increase the stack size for it to compile ```export RUST_MIN_STACK=107108864```.*


### Simulation Environment (Linux, Macos, Windows)
See [mujoco-sim/README.md](mujoco-sim/README.md) for instructions.

### Production env (Linux only)
1. Install dependencies listed in the above ```Dependencies``` section for the production environment.
2. run ```make sync``` to build/sync deps for the Unitree L2 publisher.
3. run ```make prod``` to build and run the project.

### Log replay and development tools (Linux)

#### Dependencies for log replay

   1. Git 

      This is pre installed on most Linux distros. Just set it up with either a personal access token or SSH keys.
   
   2. `make`

      Install it using your package manager. For Ubuntu/Debian, `sudo apt install build-essential`.

   3. Rerun

      Download the latest Linux binary from releases section on <https://github.com/rerun-io/rerun>. Make it executable and add it to your PATH. 

   4. clang

      Install it using your package manager. For Ubuntu/Debian, `sudo apt install libclang-dev`.

   5. Rust

      Install from [rustup.rs.](rustup.rs). After installation, switch to the nightly release using `rustup default nightly`.
   
#### Dev Tools for Linux

Recommended IDE : [VS Code](https://code.visualstudio.com)

Package Manager: `apt`, `dnf`, `pacman` etc depending on your distro.

#### Running Log Replay on Linux

   1. Clone the repository.
   2. Download a log file (linked in Discord), unzip it using `lz4 -d (filename)` and then `tar xvf (filename)` and place it in `lunabot-cu/logs`. Make sure there are no other nested directories in `lunabot-cu/logs`.
   3. In a new terminal window, in the repo directory, run `make resim`. It will take a few minutes to build and then launch the simulation in Rerun's GUI.

### Log Replay and Development tools (MacOS)

#### Dependencies for log replay

1. Git

   This is pre installed on macOS. Just set it up with either a personal access token or SSH keys.

2. `make`

   Run `xcode-select --install` if you haven't done it before. This will install `make` along with many other useful development tools.

3. Rerun

   Download the latest macOS binary from releases section on <https://github.com/rerun-io/rerun>. The latest version is 0.24.1 at the time of writing. `rerun-cli-0.24.1-x86_64-apple-darwin` if you are on an Intel Mac, or `rerun-cli-0.24.1-aarch64-apple-darwin` if you are on Apple Silicon. Make it executable with `chmod +x`. Rename the file to `rerun` and move it to `~/.local/bin`. Assuming you are using zsh, `echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc` and `source ~/.zshrc` to add it to your PATH. `which rerun` to ensure it is recognized.

4. clang

   This is pre installed on macOS.

5. Rust

   Install from [rustup.rs.](rustup.rs). After installation, switch to the nightly release using `rustup default nightly`.

#### Dev Tools for MacOS

Recommended IDE : [VS Code](https://code.visualstudio.com)

Brew - This is a useful package manager for macOS. It will be helpful. Install from [brew.sh](https://brew.sh)

#### Running

   1. Clone the repository.
   2. Download a log file (linked in Discord), unzip it using `lz4 -d (filename)` and then `tar xvf (filename)` and place it in `lunabot-cu/logs`. Make sure there are no other nested directories in `lunabot-cu/logs`.
   3. In a new terminal window, in the repo directory, run `make resim`. It will take a few minutes to build and then launch the simulation in Rerun's GUI.

### Log Replay and Development tools (Windows native)

#### Dependencies for Log Replay

   1. Git
      
      You will need to install this if you don't already have it. Download the standalone installer [here](https://git-scm.com/downloads/win). Set up git using either a personal access token or SSH keys.
   
   2. Rust
      
      Install from [rustup.rs.](rustup.rs).  Open a new PowerShell and run `rustc --version` to ensure it is recognized. After installation, switch to the nightly release using `rustup default nightly`.
   
   3. `make`
     
      The recommended method of installing this is using `choco`, see below for details. Open an admin PowerShell and run `choco install make`. After install, open a new PowerShell and run `make --version` to ensure it is recognized.

   5. LLVM/Clang
      
      Install using `choco install llvm` in an admin PowerShell. After install, open a new PowerShell and run `clang --version` to ensure it is recognized.
   
   6. C++ Build Tools
      
      You will need to install the C++ Build Tools from Microsoft. Download the installer from [Microsoft](https://visualstudio.microsoft.com/downloads/?q=build+tools). Scroll to the bottom of the page, expand "Tools for Visual Studio" and download "Build Tools for Visual Studio". This will download a Visual Studio installer. **Note that you do not need Visual Studio itself, only these build tools**. Run the installer, and in the installer window, select "Desktop development with C++" and "MacOS/Linux Development with C++" and click "Install". This will take a while to install.  
   
   7. Rerun

      Download the latest stable Windows binary, from [rerun's GitHub page](https://github.com/rerun-io/rerun/releases). The latest is 0.25.1 at this time. Download `rerun-cli-0.25.1-x86_64-pc-windows-msvc.exe` and rename it to `rerun.exe`. Move it to `C:\Windows\Program Files\Rerun` or any other directory you want. To add this to your PATH, press Windows+X, choose "System", then "Advanced system settings", then "Environment Variables". Under "System variables", select "Path" and click "Edit". Click "New" and add the path to the directory where you placed `rerun.exe`. Click OK on all the windows to close them. Open a new PowerShell window and run `rerun --version` to ensure it is recognized.

   8. 7zip ZS
      
      This is needed in order to decompress the log files which are in `.tar.lz4` format. The standard version of 7zip will not work. Download the installer from their [GitHub page](https://github.com/mcmilk/7-Zip-zstd/releases).


#### Dev Tools for Windows

Recommended IDE : [VS Code](https://code.visualstudio.com)

Package Manager: [Chocolatey](https://chocolatey.org/install). Follow the instructions for individual installation; It is one command in PowerShell. 

#### Running replay on Windows

   1. Clone the repository.
   2. Download a log file (linked in Discord), unzip it using 7zip ZS (you will need to do this twice) and place it in `lunabot-cu/logs`. Make sure there are no other nested directories in `lunabot-cu/logs`.
   3. Open a new PowerShell window, navigate to the repo directory, and run `make resim`. It will take a few minutes to build and then launch the simulation in Rerun's GUI. 


### Log replay (Docker) (Not recommended, only if you can't get native working)

1. Ensure there are valid log files in lunabot-cu/logs
2. From the repository root first build the docker container with ```docker build -t lunabot .``` then if using bash/gitbash run with ```docker run --network=host -it -v $(pwd):/workspace -v cargo-cache:/workspace/target -v cargo-registry:/usr/local/cargo/registry --name lunabot lunabot``` otherwise run with ```docker run --network=host -it -v "${PWD}:/workspace" -v cargo-cache:/workspace/target -v cargo-registry:/usr/local/cargo/registry --name lunabot lunabot```
3. Comment out the rerun init function call in lunabot-cu/src/resim.rs, and uncomment the line above with the Grpc config.
4. Start rerun on host.
5. Run replay with ```container$ make resim```

## Camera discovery

- monitors udev events and allows for easy discovery of which cameras are on which ports.

```bash
make discover-cameras
```

# Trouble Shooting
List of common problems and how to fix them can be found [here](https://github.com/utahrobotics/utah-lunabotics-2026/blob/main/TROUBLESHOOTING.md)

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

## lunabot-cu/src/tasks (not the ones in sources/ or sinks/)
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
