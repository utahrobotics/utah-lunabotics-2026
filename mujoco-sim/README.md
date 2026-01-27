# MuJoCo Simulation

## Setup
1. Follow the installation instructions [here](https://mujoco-rs.readthedocs.io/en/v2.0.x/installation.html#static-linking) to build mujoco for static linking. (you may have to leave off the release flag on the make step)

2. Ensure that the correct environment variables are set to point to the mujoco library, e.g. ```export MUJOCO_STATIC_LINK_DIR=/home/matthew_a/mujoco-rs/mujoco/build/lib```

## Running

### Option 1: With lunabot 
This option synchronizes the time step of the physics simulation to the HZ at which the copper pipeline runs, specified as ```rate_target_hz: 1000``` in the copperconfig, and ```pub static TARGET_HZ: usize = 1000;``` in sim.rs.


Additionally, starting the simulation this way launches the code for the lunabot, which is connected to the simulation environment so that the wheels will move when commanded to by the ```motor_ctrl``` task, actuators will move when commanded to by the v3_pico task, and sensor inputs for lidars and imus will be simulated by using Mujoco's rangefinder and accelerometer sensors respectively.


*Currently only the motor_ctrl and actuators work, and the localizer has a stub to just directly know where it is*

* Run the simulation by calling ```make sim```, this command may re build the entire project because mujoco requires using a different linker.


**You will also need the lunabase running to be able to control the robot:**
1. install godot https://godotengine.org/
2. build the gdext by navigating to lunabase-lib and running ```cargo build```
3. navigate to ```godot/new-lunabase```
4. launch godot editor with ```godot project.godot```
5. click one of the buttons in the top right corner to lanch the MainControl.tscn scene.


**What to use this option for:** Testing autonomy, testing manual control through the lunabase.

### Option 2: Without lunabot 
This option simply loads the artemis_arena.xml scene into the mujoco simulator with physics enabled, but does not actually run the lunabot's code, allowing for hot reloading where you can change xml files and click reload in the mujoco UI to see changes.

1. Download and extract mujoco from [here](https://github.com/google-deepmind/mujoco/releases)
2. cd into ```mujoco/bin```
3. run ```./simulate /path/to/utah-lunabotics-2026/mujoco-sim/artemis_arena.xml```

**What to use this option for:** Viewing changes to any of the xml files for the scene + model.


## Tooling

1. [obj2mjcf](https://github.com/kevinzakka/obj2mjcf)
    - Used to convert .obj mesh files into mujoco compatible xml and mesh files for importing into the scene.
    - Install instructions: 
      - For macOS:
        * Make sure mujoco is installed and added to your PATH.
        * Install `pipx` - `brew install pipx`
        * Clone the repo and cd into it
        * Set environment variables - MUJOCO_PATH=`/Applications/MuJoCo.app/Contents/Frameworks` and MUJOCO_PLUGIN_PATH=`/Applications/MuJoCo.app/Contents/MacOS/mujoco_plugin`
        * Run `pipx install .`
      - For Windows: 
        * Make sure MuJoCo is installed and `simulate` is in your PATH. 
        * Make sure cmake is installed. 
        * Make sure you are using Python 3.12. 
        * Install the package manager [Scoop](https://scoop.sh/)
        * In a new terminal session install `pipx`- `scoop install pipx` and `pipx ensurepath`.
        * Set environment variables - You can do this through PowerShell or system settings. Set `MUJOCO_PATH` and `MUJOCO_PLUGIN_PATH` to point to `Program Files/mujoco/bin` (or wherever you have installed MuJoCo).
        * Install - `pipx install obj2mjcf`. 

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
