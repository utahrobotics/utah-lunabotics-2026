# Camera Viewer

## Setup


1. Define the feeds you want to recieve in camera_layout.ron **The ip should be the ip of the lunabot.**
2. install ffmpeg and other dependencies for camera decoding using the instructions [here.](https://github.com/zmwangx/rust-ffmpeg/wiki/Notes-on-building#dependencies)
3. `cargo run --release`


## Trouble shooting


1. if you get an index out of bounds panic, delete the settings save file. (the location should be in a printed out right when the program starts).

## TODO:


1. Auto reconnect would be nice.


