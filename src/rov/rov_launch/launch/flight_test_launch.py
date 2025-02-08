from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(package="rov_flight_controller",
            namespace="",
            executable="flight_controller",
            name="rov_flight_controller",
            parameters=[os.path.join(get_package_share_directory("rov_flight_controller"), "config", "params.yaml")]),
        Node(package="rov_control",
            namespace="",
            executable="rov_control",
            name="rov_control",
            parameters=[os.path.join(get_package_share_directory("rov_control"), "config", "default.yaml")]),
        Node(package="rov_pca9685",
            namespace="",
            executable="pca9685_node",
            name="rov_pca9685")
        ])