#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="flight_controller",
            executable="flight_controller",
            name="flight_controller",
            parameters=[
            os.path.join(get_package_share_directory("flight_controller"), "config", "params.yaml")
            ],
            output='screen')
    ])