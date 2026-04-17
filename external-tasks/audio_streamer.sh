#!/bin/bash
audio_device=""
if [ "$1" == "new_pc" ]; then
    audio_device="alsa_input.pci-0000_e5_00.6.analog-stereo.2"
elif [ $1 == "old_pc" ]; then
    audio_device="alsa_input.pci-0000_34_00.6.analog-stereo"
else
    echo "specify new_pc or old_pc"
    exit
fi
echo $audio_device
gst-launch-1.0 pulsesrc device=$audio_device ! audioconvert ! audioresample ! opusenc bitrate=48000 audio-type=voice frame-size=10 ! gdppay ! tcpserversink host=0.0.0.0 port=5000