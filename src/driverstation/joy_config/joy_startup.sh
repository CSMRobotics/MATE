#!/bin/bash
source install/setup.bash
ros2 run joy joy_node --ros-args --params-file src/driverstation/joy_config/joy_left_params.yaml --remap joy:=joy_left & ros2 run joy joy_node --ros-args --params-file src/driverstation/joy_config/joy_right_params.yaml --remap joy:=joy_right 