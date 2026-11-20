from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='thrustmaster_joy_node',
            parameters=[{
                'device_name': "Thrustmaster T.Flight Stick X"
            }],
            remappings=[
                ('/joy', '/thrustmaster_joy')
            ]
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='logi_joy_node',
            parameters=[{
                'device_name': "Logitech Extreme 3D pro"
            }],
            remappings=[
                ('/joy', '/logi_joy')
            ]
        )
    ])