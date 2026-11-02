gst-launch-1.0 rtspsrc location=rtsp://192.168.0.42:8554/camera0 latency=0 ! decodebin ! videoconvert ! ximagesink sync=false
