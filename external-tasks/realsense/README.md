# Realsense

#### Outputs
Publishes point clouds on ```"realsense/{serial num}/cloud```
Publishes obstacle map on ```realsense/{serial num}/occupancy```

## Order of operations

1. The realsense external process is started by the launcher in lunabot-cu/src/main.rs
2. Waits until a depth camera with the serial number defined in src/constants.rs is connected.
3. DepthCameraTask.process_camera_session() is called once a realsense connects, and the misc/thalassic crate is used to start a depth projector shader pipeline.
4. The depth camera task subscribes to ```localizer/realsense_isometry``` and feeds the depth camera's isometry to the depth projector so the point clouds are projected from the right place in space.
5. The point clouds created by the depth projector are fed into the occupancy grid pipeline from the misc/thalassic crate.
6. Occupancy grids are published on an iceoryx2 node+service to be subscribed to by lunabot-ai2 to be used for path finding.


## About occupancy grid generation
See the misc/thalassic crate readme for details on how the obstacle detection works.

## Configuring the occupancy grid generation params
All the params for obstacle detection are in constants.rs \\
A description of what they do can be found in misc/thalassic/README.md
