from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(node_name="rov_cameras",
             executable="rov_cameras")
    ])