import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            #cmd=["ros2", "run", "camera_stream", "rtsp", "\"videotestsrc ! videoconvert ! x264enc ! rtph264pay name=pay0 pt=96\""], # This is a test pipeline for non-nvarguscamerasrc compatible systems
            cmd=["ros2", "run", "camera_stream", "rtsp", "\"nvarguscamerasrc ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96\""],
            output='both',
            shell=True
        )
    ])