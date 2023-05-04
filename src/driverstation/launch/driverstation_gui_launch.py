from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package="driverstation_gui",
             namespace="driverstation_gui",
             executable="driverstation_gui",
             name="driverstation_gui"),
        Node(package="joy",
             namespace="joy",
             executable="joy_node",
             name="joy")
    ])