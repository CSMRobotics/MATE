#!/bin/bash
set -e
export _WS=/opt/ros/humble

ros_env_setup="$_WS/setup.bash"
echo "sourcing   $ros_env_setup"
source "$ros_env_setup"

echo "ROS_ROOT   $ROS_ROOT"
echo "ROS_DISTRO $ROS_DISTRO"

exec "$@"