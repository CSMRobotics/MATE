import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('uart_manager'),
        'config',
        'test_config.yaml'
    )

    log_level = LaunchConfiguration('log_level')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value='INFO'
        ),

        Node(
            package="uart_manager",
            executable="uart_manager",
            name="uart_manager",
            output="screen",
            parameters=[config_file],
            arguments=[
                '--ros-args',
                '--log-level', log_level
            ]
        ),

    ])