# MuJoCo Simulation

## Setup
1. Follow the installation instructions [here](https://mujoco-rs.readthedocs.io/en/v1.5.x/installation.html#static-linking) to build mujoco for static linking, except replace the two cmake commands listed there with this:
```bash
cmake -B build -S . -DBUILD_SHARED_LIBS:BOOL=OFF -DMUJOCO_HARDEN:BOOL=OFF -DCMAKE_BUILD_TYPE:STRING=Release -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=ON -DMUJOCO_BUILD_EXAMPLES:BOOL=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_EXE_LINKER_FLAGS:STRING=-Wl,--no-as-needed
```
and this:
```bash
cmake --build build --parallel --target simulate libsimulate --config=Release
```
2. Ensure that the correct environment variables are set to point to the mujoco library, e.g. ```export MUJOCO_STATIC_LINK_DIR=/home/matthew_a/mujoco-rs/mujoco/build/lib```

## Running

### Option 1: With lunabot 
This option synchronizes the time step of the physics simulation to the HZ at which the copper pipeline runs, specified as ```rate_target_hz: 1000``` in the copperconfig, and ```pub static TARGET_HZ: usize = 1000;``` in sim.rs.


Additionally, starting the simulation this way launches the code for the lunabot, and will eventually be connected to the simulation environment so that the wheels will move when commanded to by the ```motor_ctrl``` task, actuators will move when commanded to by the v3_pico task, and sensor inputs for lidars and imus will be simulated by using Mujoco's rangefinder and accelerometer sensors respectively.

* Run the simulation by calling ```make sim```, this command may re build the entire project because mujoco requires using a different linker.

**What to use this option for:** Testing autonomy, testing manual control through the lunabase.

### Option 2: Without lunabot 
This option simply loads the artemis_arena.xml scene into the mujoco simulator with physics enabled, but does not actually run the lunabot's code, allowing for hot reloading where you can change xml files and click reload in the mujoco UI to see changes.

1. cd into the directory where you cloned mujoco-rs
2. cd into ```mujoco/build/bin```
3. run ```./simulate ./simulate /path/to/utah-lunabotics-2026/mujoco-sim/artemis_arena.xml```

**What to use this option for:** Viewing changes to any of the xml files for the scene + model.