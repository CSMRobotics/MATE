# import os
# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# import re
# import subprocess

def generate_launch_description():
    ld = LaunchDescription()

    # robot_control_config = os.path.join(
    #     get_package_share_directory("robot_control"), "config", "default.yaml"
    # )

    ld.add_action(
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
        )
    )

    return ld