# Lunabot

*This repo is in the earlyish stages of a rewrite migrating to the copper framework from last years code base: github.com/utahrobotics/lunadev-2025*

## Architecture Overview
<img width="3880" height="946" alt="graphviz(4)" src="https://github.com/user-attachments/assets/a403ef4e-9198-4120-a0da-43c36a812f3b" />

##### Check copperconfig.ron to see the definitions of all the tasks running and the datatypes passed between tasks.


## Dependencies

### Production env (Linux only)
1. See dockerfile, anything installed there is a dependency for running the robot in production.

### Log Replay (Unix)

1. ```make``` command 
2. git - just google how to install this
3. rerun - download binary from their github releases page and add "rerun" command to Path.
4. libclang-dev
5. rust - install from rustup.rs (select nightly version)

NOTE: If you are on macos you can install git, libclang, and make by running ```xcode-select --install```

### Log Replay (Docker)
1. ```docker build -t lunabot . && docker run --network=host -it lunabot /bin/bash```
2. Find the docker container id with ```host$ docker container ls```
3. Copy log files from host to container ```host$ docker cp  path/to/lunabot-cu/logs container_id:/workspace/lunabot-cu/logs```
4. Comment out the rerun init viz line in lunabot-cu/main.rs, and uncomment the line above with the Grpc configuration, replacing the ip with host.docker.internal
4. Start rerun on host.
5. Run replay ```container$ make resim``` 

### Optional Dependencies

1. **cubuild** - Enhanced error messages for Copper macros
   ```bash
   https://github.com/copper-project/copper-rs/tree/master/support/cargo_cubuild
   ```
2. iox2 cli tool for seeing active iceoryx2 nodes and services.
3. cargo flamegraph + perf for profiling
4. gdb
5. lz4 for compressing logs


## Running

### Production env
1. On some machines you have to increase the stack size for it to compile ```export RUST_MIN_STACK=107108864```.
2. run ```make sync``` to build the Unitree L2 publisher.
3. run ```make prod``` to build and run the project.

### Log replay
1. Ensure there are valid log files in lunabot-cu/logs
1. run ```make resim```


### Help
```bash
make help # see commands for building and running
```

## Camera discovery

- monitors udev events and allows for easy discovery of which cameras are on which ports.

```bash
make discover-cameras
```
