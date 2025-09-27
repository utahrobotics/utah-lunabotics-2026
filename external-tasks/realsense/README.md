# Realsense

#### Outputs
Publishes depth frames on ```realsense/{serial num}/depth```
Publishes imu frames on ```realsense/{serial num}/imu```

## Order of operations

1. The realsense external process is started by the launcher in lunabot-cu/src/main.rs
2. Waits until a depth camera with the serial number defined in src/constants.rs is connected.
3. Opens the camera and starts publishing depth frames and imu data over iceoryx2