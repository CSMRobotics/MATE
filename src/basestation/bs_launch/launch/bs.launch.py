from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    bs_flask = Node(
        package="bs_flask",
        node="bs_flask"
    )

    basestation_estop = Node(
        package="basestation_estop",
        node="basestation_estop"
    )

    ld.add_action(bs_flask)
    ld.add_action(basestation_estop)

    return ld