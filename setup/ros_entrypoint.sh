#!/bin/bash
set -e
export CSM_WS=/opt/ros/CSM_MATE_WS

ros_env_setup="$CSM_WS/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

exec "$@"