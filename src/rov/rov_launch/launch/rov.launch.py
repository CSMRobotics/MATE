from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    gpio = Node(
        package="rov_gpio",
        executable="rov_gpio"
    )
    bar30 = Node(
        package="rov_bar30",
        executable="bar30"
    )
    bno055 = Node(
        package="rov_bno055",
        executable="bno055_node"
    )
    flight_controller = Node(
        package="rov_flight_controller",
        executable="flight_controller",
        parameters=[os.path.join(get_package_share_directory("rov_flight_controller"), "config", "params.yaml")]
    )
    # manipulator_control = Node()
    pca9685 = Node(
        package="rov_pca9685",
        executable="pca9685_node"
    )
    cameras = Node(
        package="rov_cameras",
        executable="rov_cameras",
        name="rov_cameras",
        parameters=[os.path.join(get_package_share_directory("rov_cameras"), "config", "params.yaml")],
    )
    control = Node(
        package="rov_control",
        executable="rov_control",
        name="rov_control",
        parameters=[os.path.join(get_package_share_directory("rov_control"), "config", "default.yaml")]
    )
    manipulator_controller = Node(
        package="rov_manipulator_controller",
        executable="manipulator_controller",
        parameters=[os.path.join(get_package_share_directory("rov_manipulator_controller"), "config", "params.yaml")]
    )

    led_controller = Node(
        package="rov_led_controller",
        executable="led_controller",
        # parameters=[os.path.join(get_package_share_directory("rov_led_controller"), "config", "params.yaml")]
    )

    tsys01 = Node(
        package="rov_tsys01",
        executable="tsys01"
    )

    ld.add_action(bno055)
    ld.add_action(flight_controller)
    ld.add_action(pca9685)
    ld.add_action(cameras)
    ld.add_action(control)
    ld.add_action(bar30)
    ld.add_action(gpio)
    ld.add_action(manipulator_controller)
    ld.add_action(led_controller)
    ld.add_action(tsys01)

    print("NONONONo Robobobo alive")
    return ld
