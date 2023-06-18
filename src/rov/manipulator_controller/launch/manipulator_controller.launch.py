#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="manipulator_controller",
            executable="manipulator_controller",
            name="manipulator_controller",
            parameters=[
            os.path.join(get_package_share_directory("manipulator_controller"), "config", "params.yaml")
            ])
    ])