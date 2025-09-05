# Lunabot

*This repo is in the early stages of a rewrite migrating to the copper framework from last years code base: github.com/utahrobotics/lunadev-2025*

## Architecture Overview
<img width="4238" height="1064" alt="graphviz(2)" src="https://github.com/user-attachments/assets/1fed9b5c-01c8-4025-a19f-879566554475" />

##### Check copperconfig.ron to see the definitions of all the tasks running and the datatypes passed between tasks.


## Dependencies

### Required Dependencies (See non linux setup for other targets, or if you only need log replay / simulation)

1. See dockerfile, anything installed there is a dependency.


### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   ```bash
   https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild
   ```

2. Rerun - visualize data produced by the robot.
3. iox2 cli tool for seeing active iceoryx2 nodes and services.




## Building and Running
1. Install rust from rustup.rs
2. ```rustup default nightly```
3. On some machines you have to increase the stack size for it to compile ```export RUST_MIN_STACK=107108864```.
4. Install dependencies listed in the Dockerfile.
5. run ```make sync``` to build the Unitree L2 publisher.
6. run ```make prod``` to build and run the project.

```bash
make help # see commands for building and running
```

## Camera discovery

- monitors udev events and allows for easy discovery of which cameras are on which ports.

```bash
make discover-cameras
```


## TODO: everything logging related
1. copper log macros only print in debug builds (I likely will change this)
2. console monitor stuff
3. rerun stuff

## Non Linux Log Replay Quick Setup
1. ```curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh``` (select the nightly version)
2. install lapack and openblas using vcpkg or brew depending on if you are on windows or macos
3. use ```make resim``` for log replay