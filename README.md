# Lunabot

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

