from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="flight_controller",
            namespace="",
            executable="flight_controller",
            name="flight_controller"),
        Node(package="rov_control",
            namespace="",
            executable="rov_control",
            name="rov_control"),
        Node(package="pca9685",
            namespace="",
            executable="pca9685_node",
            name="pca9685"),
        Node(package="bno055",
            namespace="",
            executable="bno055",
            name="bno055"
        ])
