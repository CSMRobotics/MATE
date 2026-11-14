import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_config_file = os.path.join(
        get_package_share_directory('rov_cameras'),
        'config',
        'camera_config.yaml'
    )

    return LaunchDescription([

        Node(
            package="rov_cameras",
            executable="rov_cameras",
            name="rov_cameras",
            output="screen",
            parameters=[camera_config_file]
        ),

    ])