import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('uart_manager'),
        'config',
        'test_config.yaml'
    )

    return LaunchDescription([

        Node(
            package="uart_manager",
            executable="uart_manager",
            name="uart_manager",
            output="screen",
            parameters=[config_file]
        ),

    ])