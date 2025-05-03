#!/bin/bash
gst-launch-1.0 rtspsrc location=rtsp://10.42.0.2:8554/video latency=0 ! decodebin ! videoconvert ! autovideosink
