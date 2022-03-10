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
            package="robot_control",
            executable="robot_control",
            name="robot_control",
            # parameters=[robot_control_config, override_config],
            # remappings=[("/robot/cmd_velocity", "/odrive/cmd_velocity")],
        )
    )

    return ld