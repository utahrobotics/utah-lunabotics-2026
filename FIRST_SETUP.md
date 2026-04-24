# First Time Setup Instructions
*This guide assumes basic linux knowledge. i.e. where to put udev rules, how to add a user to a group, how to use ssh*

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
	1. if you only have one plugged in or something, just make stuff up for the other serial numbers cause it doesn't matter
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

<br/>

**Trouble Shooting:** <br/>
- the t265 fails to boot with some error message:
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
5. Update the realsense_subscriber task's copperconfig.ron to have the serial number of the upper depth camera, and the corresponding node name defined in `robot_layout/lunabot.ron.`
```ron
config: {
	"serial_num": "341222301328",
	"camera_node": "upper_depth_camera"
},
```

6. If you wish to stream the rgb frames from the depth camera, run `make discover-cameras`, then unplug and replug the realsense and take note of the ports that pop up. There should be two different ports that pop up, each with a few indexes. The RGB should be index 0 on the port that mentions "1.3". (I believe this is because the realsense offers usb 2 and usb 3 and the rgb is on 3). 
Update the `d456_rgb` task's config to have that port. 

<br/>

**Realsense Troubleshooting** <br/>

1. "**d456_rgb reported no frames after 500 ms**" message or if the rgb feed doesn't seem to work: try running `ffplay /dev/videoX` where /dev/videoX is the port you think corresponds with the rgb. If that is the rgb feed, you will see it playing, then if that works and there are no errors with the d456_rgb task make sure your camera feed ip and port are set correctly in the copperconfig, and in the multiplexed camera viewer config.
2. **No depth frames in 500 ms**: If there are no depth frames recieved, first run `make kill` to kill all lunabot related processes, then cd into external_tasks/realsense and directly run `cargo run --release` from there, then plug in the realsense. If you see a few messages about the realsense popping up and publishing, it should be working. Double check that the serial number in the realsense_subscriber tasks config is correct. 

### RGB cameras

1. Plug in your rgb cameras, and make note of their positions on the robot with respect to the center like you did the other cameras.
2. Run `make discover-cameras` and start un plugging and re plugging the cameras until you figure out the ports that the cameras are on.
	1. these will need to be the same usb ports you always plug the cameras into, same every time.
3. Update one of the entries to the includes array (for one of the camera_template_gstreamer ones) in copperconfig.ron, use [Naj's focal length estimator](https://github.com/utahrobotics/focal-length-estimator) or some other tool to figure out the intrinsics.
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
4. make sure there is a node in the robot layout with the format cam_<id> using the id you specified in the copperconfig

<br/>

**Trouble Shooting RGB Cameras:**
<br/>

1. cam_x no frames in 500 ms:

- Check the port is right, use the `make discover-cameras` and `ffplay /dev/videox` commands to help
- Run `v4l2-ctl --list-formats-ext` to list the available capture profiles and then ensure that your width and height specified in the config are compatable with at least 30 fps for the MJPG type.


# Network Stuff

### Prerequisites
1. Obtain ssh access by putting your public key in the right places. (probably .ssh/authorized_keys)
2. Ensure you are on the same wifi network as the robot.
3. ssh in, go to the root of this repo, and run `make prod`

### Base station comms
1. Just type in the ip of the robot in the base station and hit connect
2. Might be helpful to log into the wifi router and set a static ip for the robot

### Camera streams

1. follow the installation instructions for the [multiplexed camera viewer](https://github.com/utahrobotics/utah-lunabotics-2026/tree/main/misc/multiplexed-camera-viewer) crate in misc.
	1. You will need to specify in the config for the camera viewer what the robots ip is, and the ports for the streams you want to connect to 
2. Ctrl-f for `tcp_port` in copperconfig.ron to whatever you want for each camera.
- Note that for all the camera streaming, the camera stream is a server hosted on the robots pc, and the camera multiplexed viewer is the client.

#### Camera stream trouble shooting
1. Check the task error messages (in lunabase, or from the periodic task error prints) for any messages about the camera feeds.
2. Consult "trouble shooting RGB cameras" section of this guide.
3. if there are no error messages in the lunabase mentioning the cameras, then double check that your ports are configured correctly in the multiplexed camera viewer config, and the copperconfig.ron.
4. If the feeds are frozen, try clicking the red "Disable" button in the top left of the multiplexed viewer, then re enabling the streams and seeing if they reconnect.


### Network Trouble shooting
*Cant find ip of the robot*: <br/>
1. Ensure you are on the same wifi network as the robot, make sure the robot pc is powered on and has had a minute or two to boot then run `nmap -p 22 192.168.0.0/24` to discover devices with an open ssh port on the network.

# Vescs
1. Plug the vescs into the pc
2. Change the motor_ctrl config to have the right can id's and motor masks.
3. Run make prod, disengage e-stop and make sure you see a message that the motor port was opened.

### Vesc troubleshooting
1. Call sebastion.
2. If there arent any lights on the vesc, that means it isnt getting power or it blew up.
3. Check make prod stdout for any messages saying "Opened motor port", or for any permission denied messages on ttyacmX


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
4. Only the prime pico needs to be attached to the computer over micro usb, the secondary pico should not be attached.
5. Make sure all 3 red LED's on the board the prime pico is attached to are lit. If not all of them are lit that means something isn't getting powered (maybe the secondary pico?).
6. in copperconfig.ron set the config for the pico task to have `teri_mode: false`




# Permissions

1. **Add your user to the following groups: dialout, render, adm**
2. **Add the [realsense udev rules to the correct directory](https://github.com/realsenseai/librealsense/blob/master/config/99-realsense-libusb.rules)**
3. Add the udev rules in misc/usb-reset/99-usb-reset-rules.rules