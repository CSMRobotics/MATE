from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(node_name="rov_cameras",
             executable="rov_cameras",
        ExecuteProcess(
            #cmd=["ros2", "run", "camera_stream", "rtsp", "\"videotestsrc ! videoconvert ! x264enc ! rtph264pay name=pay0 pt=96\""], # This is a test pipeline for non-nvarguscamerasrc compatible systems
            cmd=["ros2", "run", "camera_stream", "rtsp", "\"nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96\""],
            output='both',
            shell=True
        )
    ])
