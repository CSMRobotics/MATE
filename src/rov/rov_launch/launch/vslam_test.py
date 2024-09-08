
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import re


def get_cam_index(keyword):
    try:
        output = subprocess.check_output("ls /dev/v4l/by-id -lah | grep " + keyword, shell=True)
        pattern = "video\\d+"
        result = re.findall(pattern, str(output))
        new_results = []
        for res in result:
            res = res.replace("video", "")
            new_results.append(int(res))
        return min(new_results)
    except Exception:
        return None


def get_cam_index_from_list_of_strings(search_strings):
    for search_string in search_strings:
        cam_index = get_cam_index(search_string)
        if cam_index:
            return cam_index

    return 0


def get_vslam_index():

    vslam_camera_search_strings = [
        "usb-Sonix_Technology_Co.__Ltd._292A-AR0230_SN0001-video-index0",
        # "usb-H264_USB_Camera_H264_USB_Camera_2020032801-video-index0", # old cam - name conflicts with windshield
    ]

    return get_cam_index_from_list_of_strings(vslam_camera_search_strings)


def generate_launch_description():
    ld = LaunchDescription()
    # VSLAM SETUP
    vslam_pkg_path = get_package_share_directory("rov_launch")
    vslam_node_yaml = os.path.join(vslam_pkg_path, "config", "vslam_node.yaml")

    ld.add_action(
        Node(
            package="image_publisher",
            executable="image_publisher",
            name="vslam_image_publisher",
            parameters=[{
                # "camera_index": get_vslam_index(),
                "source_name": "video", 
                "media_path": os.path.join(vslam_pkg_path, "media", "zac_can_see_your_soul.mp4")
            }],
            remappings=[("image_output", "vslam_image")],
        )
    )

    ld.add_action(
        Node(
            package="vslam_node",
            executable="vslam_node",
            name="vslam_node",
            parameters=[vslam_node_yaml, {"enable_preview": True, "enable_video_feedback": True}],
        )
    )

    return ld
