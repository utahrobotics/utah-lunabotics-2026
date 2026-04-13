#!/bin/bash
gst-launch-1.0 tcpclientsrc host=$1 port=$2 ! gdpdepay ! opusdec ! audioconvert ! autoaudiosink sync=false