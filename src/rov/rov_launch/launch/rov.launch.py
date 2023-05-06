from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    bno055 = Node(
        package="bno055",
        executable="bno055_node"
    )
    flight_controller = Node(
        package="flight_controller",
        executable="flight_controller",
        arguments= [os.path.join(get_package_share_directory("flight_controller"), "config", "params.yaml")]
    )
    # manipulator_control = Node()
    pca9685 = Node(
        package="pca9685_node",
        executable="pca9685_node"
    )
    cameras = Node(
        package="rov_cameras",
        executable="rov_cameras"
    )
    control = Node(
        package="rov_control",
        executable="rov_control",
        name="rov_control",
        parameters=[os.path.join(get_package_share_directory("rov_control"), "config", "default.yaml")],
    )
    estop = Node(
        package="rov_estop",
        executable="rov_estop"
    )
    # statistics = Node()

    ld.add_action(bno055)
    ld.add_action(flight_controller)
    ld.add_action(pca9685)
    ld.add_action(cameras)
    ld.add_action(control)
    ld.add_action(estop)

    return ld