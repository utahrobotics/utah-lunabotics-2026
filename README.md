# Lunabot

*This repo is in the early stages of a rewrite migrating to the copper framework from last years code base: github.com/utahrobotics/lunadev-2025*

## Architecture Overview
<img width="4000" height="1114" alt="graphviz(1)" src="https://github.com/user-attachments/assets/9c1381c1-8286-4561-a0cc-5f87f72bd1b4" />

##### Check copperconfig.ron to see the definitions of all the tasks running and the datatypes passed between tasks.


## Dependencies

### Required Dependencies

1. See dockerfile, anything installed there is a dependency.


### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   ```bash
   https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild
   ```

2. Rerun - visualize data produced by the robot.




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

