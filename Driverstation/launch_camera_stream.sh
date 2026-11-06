#!/bin/bash

# Run the front camera stream in the background
gst-launch-1.0 rtspsrc location=rtsp://192.168.0.42:8554/front latency=0 ! decodebin ! videoconvert ! ximagesink sync=false &

# Run the bottom camera stream in the background
gst-launch-1.0 rtspsrc location=rtsp://192.168.0.42:8554/bottom latency=0 ! decodebin ! videoconvert ! ximagesink sync=false &

# Wait for both background streams to finish
wait