
# Sensors

### T265/1
1. Plug in the t265
2. run rs-enumerate-devices
3. take note of the serial number
4. measure the exact position of the t265 with that serial number with respect to the center of the robot
	1. you can pick any point to be the center really, it just has to be the same point for all the sensors
5. Update robot_layout/lunabot.ron with the actual position of the sensor, with the node name being the serial number, for example:

```ron
{
//t265 rear
	"name": Some("929122111514"),
	"origin": [0.065,0.20,0.0],
	"euler": [180.0,0.0,90.0],
}
```
6. Update the t265_subscriber task's config in copperconfig.ron to have the correct serial numbers for left right and rear.
	1. if you only have one plugged in or something, just make shit up for the other serial numbers cause it doesn't matter
7. Edit the three apriltag detector tasks  for the t265's to have the right info:
```ron
(
	id: "detector_cam_t265_left",
	type: "cu_apriltag::AprilTags",
	config: {
		"family": "tag36h11",
		"tag_size": 0.145,
		// the fx/fy/cx/cy dont matter if you specify the distortion params
		"fx": 1,
		"fy": 1,
		"cx": 960.84,
		"cy": 600.819,
		// change this to be the right serial number
		"camera_id": "142122111023",
		// you can keep this the same, in between t265's there isn't much difference
		"distortion_path": "camera_configs/t265_distortion.ron",
	},
(
```

**Trouble Shooting**
- the t265 fails to boot with some fuck ass error message:
	- unplug and replug it, then try again.
	- sometimes having realsense devices on a usb hub causes them to behave weird, so if you have them directly plugged in that might work better
	- I am working on a way to power cycle them without re plugging them in case this happens during competition.
### Depth Camera (d465)
1. make sure you are using the fancy dust protection rated d456 not the d455
2. connect it to the PC with a cable and port that supports usb 3.0 or higher
3. run `rs-enumerate-devices` and take note of the serial number
4. measure the position of the depth camera with respect to the center of the robot, then update robot_layout/lunabot.ron with a node that describes the depth cameras position, for example:
```ron
{
	"name": Some("upper_depth_camera"),
	"origin": [0.89, 0.0, 0.31],
	"euler": [0.0, 28.0, 0.0]
},
```
5. update the realsense_subscriber task's copperconfig.ron to have the serial number of the upper depth camera, and the right node name in the config (maybe I should have just kept the pattern of using the serial number as the node name but ehhhh I cant be bothered to change it cause its a minor thing)
```ron
config: {
	"serial_num": "341222301328",
	"camera_node": "upper_depth_camera"
},
```

### RGB cameras

1. Plug in your rgb cameras, and make note of their positions on the robot with respect to the center like you did the other cameras.
2. Run `make discover-cameras` and start un plugging and re plugging the cameras until you figure out the ports that the cameras are on.
	1. these will need to be the same usb ports you always plug the cameras into, same every time.
3. Add or update an entry to the includes array in copperconfig.ron, use [Naj's focal length estimator](https://github.com/utahrobotics/focal-length-estimator) or some other tool to figure out the intrinsics.
	1. You will need an apriltag to estimate the intrinsics using Naj's tool
```ron
params: {
	"id":"back", // pick some id for the camera
	"port":"pci-0000:00:14.0-usb-0:5.3:1.0", // use the port you discovered from the make discover-cameras cmd
	
	"fx":667.0,
	"fy":667.0,
	"cx":960.84,
	"cy":600.819,
	"width":1920,
	"height":1200,
	"tcp_host":"0.0.0.0",
	"tcp_port":4000,
},
```
4. make sure there is a node in the robot layout with the same id you specified in the copperconfig

# Network Stuff

### Base station comms
1. Just type in the ip of the robot in the base station and hit connect
2. Might be helpful to log into the wifi router and set a static ip for the robot

### Camera streams

1. Install ffmpeg on your machine, then run the [multiplexed camera viewer](https://github.com/utahrobotics/utah-lunabotics-2026/tree/main/misc/multiplexed-camera-viewer)
	1. You will need to specify in the config for the camera viewer what the robots ip is, and the ports for the streams you want to connect to 

# Vescs
1. Call Sebastion
2. Change the motor_controller config to have the right can id's and motor masks.

# Actuators
1. Attach debug probe and plug the debug probe into your machine, as well as have the pico attached over usb.

**For Teri:** 
1. Nagivate to the `embedded-legacy/v3pico` directory.
2. Run `cargo run --release` to flash the pico.
3. in copperconfig.ron set the config for the pico task to have `teri_mode: true`

<br/>

**For New Robot:**
1. Navigate to embedded directory.
2. run `cargo run --release --bin pico-prime` to flash the pico that controlls the actuators.
3. attach the other pico then run `cargo run --release --bin pico-secondary` to flash it.
