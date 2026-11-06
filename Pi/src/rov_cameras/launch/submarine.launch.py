from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="rov_cameras",
            executable="rov_cameras",
            name="rov_cameras",
            output="screen",
            parameters=[{
                # RTSP configuration
                "rtsp_port": 8554,

                # Camera configuration found with 'v4l2-ctl --list-devices' then 'v4l2-ctl -d /dev/video0 --list-formats-ext'
                "camera_names": [
                    "H264 USB Camera: H264 USB Camer",
                    "HD USB Camera: HD USB Camera",
                ],

                "camera_widths": [
                    "default",
                    "1280",
                ],

                "camera_heights": [
                    "default",
                    "720",
                ],  

                "camera_framerates": [
                    "default",
                    "30",
                ],

                # Corresponding RTSP endpoints
                "stream_endpoints": [
                    "front",
                    "bottom",
                ],
            }],
        ),

    ])