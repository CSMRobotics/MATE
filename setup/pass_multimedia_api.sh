#!/bin/bash

echo "dir,  /usr/src/jetson_multimedia_api" | sudo tee --a /etc/nvidia-container-runtime/host-files-for-container.d/l4t.csv > /dev/null