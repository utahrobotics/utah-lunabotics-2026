# MuJoCo Simulation

## Setup
1. Follow the installation instructions [here](https://mujoco-rs.readthedocs.io/en/v2.0.x/installation.html#static-linking) to build mujoco for static linking, except replace the two cmake commands listed there with this:
```bash
cmake -B build -S . \
  -DBUILD_SHARED_LIBS:BOOL=OFF \
  -DMUJOCO_HARDEN:BOOL=OFF \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -DCMAKE_INTERPROCEDURAL_OPTIMIZATION:BOOL=OFF \
  -DMUJOCO_BUILD_EXAMPLES:BOOL=OFF \
  -DGLFW_BUILD_WAYLAND=ON \
  -DGLFW_BUILD_X11=OFF
```
and this:
```bash
cmake --build build --parallel --target glfw libmujoco_simulate --config=Release
```
2. Ensure that the correct environment variables are set to point to the mujoco library, e.g. ```export MUJOCO_STATIC_LINK_DIR=/home/matthew_a/mujoco-rs/mujoco/build/lib```

## Running

### Option 1: With lunabot 
This option synchronizes the time step of the physics simulation to the HZ at which the copper pipeline runs, specified as ```rate_target_hz: 1000``` in the copperconfig, and ```pub static TARGET_HZ: usize = 1000;``` in sim.rs.


Additionally, starting the simulation this way launches the code for the lunabot, which is connected to the simulation environment so that the wheels will move when commanded to by the ```motor_ctrl``` task, actuators will move when commanded to by the v3_pico task, and sensor inputs for lidars and imus will be simulated by using Mujoco's rangefinder and accelerometer sensors respectively.


*Currently only the motor_ctrl and actuators work, and the localizer has a stub to just directly know where it is*

* Run the simulation by calling ```make sim```, this command may re build the entire project because mujoco requires using a different linker.

**What to use this option for:** Testing autonomy, testing manual control through the lunabase.

### Option 2: Without lunabot 
This option simply loads the artemis_arena.xml scene into the mujoco simulator with physics enabled, but does not actually run the lunabot's code, allowing for hot reloading where you can change xml files and click reload in the mujoco UI to see changes.

1. cd into the directory where you cloned mujoco-rs
2. cd into ```mujoco/build/bin```
3. run ```./simulate /path/to/utah-lunabotics-2026/mujoco-sim/artemis_arena.xml```

**What to use this option for:** Viewing changes to any of the xml files for the scene + model.


## Tooling

1. [obj2mjcf](https://github.com/kevinzakka/obj2mjcf)

## Development 

### Adding a Mesh to the scene
1. Use the obj2mjcf to segment the model into a collection of convex meshs, and generate the files needed for importing it into the scene: ```obj2mjcf --compile-model --save-mjcf --decompose --overwrite --obj-dir .   --coacd-args.threshold 0.01```
2. Rename DefaultMaterial in the generated xml file to something else.
3. Delete the ```<default>...<\default>``` element.
4. Move all the generated files in the directory to mujoco-sim/meshes
5. Import the item into the scene by adding

 ```xml
 <include file="meshes/model_name/model_name.xml" />
 ```
To artemis_arena.xml, or use [attach](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html#body-attach).

6. Set the [joint type](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint)
7. Make sure the generated meshes look fine by changing the group of the visual class to 3 and the group of the collision class to 2 in artemis_arena.xml, because that will make the collision meshes visible in the simulator UI:
```xml
<default>
    <default class="visual">
      <geom group="2" type="mesh" contype="0" conaffinity="0" />
    </default>
    <default class="collision">
      <geom group="3" type="mesh" />
    </default>
</default>
 ```


### Learning Resources

1. Mujoco-rs docs
* Guide: https://mujoco-rs.readthedocs.io/en/v2.0.x/
* API docs: https://docs.rs/mujoco-rs/latest/mujoco_rs/
* Mujoco-rs examples: https://github.com/davidhozic/mujoco-rs/tree/main/examples 
2. [Mujoco Docs](https://mujoco.readthedocs.io/en/3.3.7/overview.html)
3. [Mujoco XML reference](https://mujoco.readthedocs.io/en/3.3.7/XMLreference.html)
4. [Mujoco model examples](https://github.com/google-deepmind/mujoco_menagerie/tree/main)
