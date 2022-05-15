import threading
from time import sleep
import cv2
import rclpy
from rclpy.node import Node
import signal

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import glob

FPS = float(30.0)

class ROVCameras(Node):
    def __init__(self):
        super().__init__("rov_cameras")
        self.active_cameras = set()
        self.disconnected_cameras = set()
        self.construct_cameras()
        self.br = CvBridge()
        self.string_pub = self.create_publisher(String, "test", 1)

        self.camera_publishers = {
            "cam0":self.create_publisher(Image, "cam0_image", 10),
            "cam1":self.create_publisher(Image, "cam1_image", 10),
            "cam2":self.create_publisher(Image, "cam2_image", 10),
            "cam3":self.create_publisher(Image, "cam3_image", 10)
        }

        self.camera0_control_subscriber = self.create_subscription(Bool, "cam0_control", self.cam0_control, 10)
        self.camera1_control_subscriber = self.create_subscription(Bool, "cam1_control", self.cam1_control, 10)
        self.camera2_control_subscriber = self.create_subscription(Bool, "cam2_control", self.cam2_control, 10)
        self.camera3_control_subscriber = self.create_subscription(Bool, "cam3_control", self.cam3_control, 10)

        self.create_timer(1/FPS, self.cams)

    def cam0_control(self, msg:Bool):
        self.controlCam(0, msg)

    def cam1_control(self, msg:Bool):
        self.controlCam(1, msg)

    def cam2_control(self, msg:Bool):
        self.controlCam(2, msg)

    def cam3_control(self, msg:Bool):
        self.controlCam(3, msg)

    def construct_cameras(self):
        self.cameras = []
        self.captured_cam_events = []
        for camera in glob.glob("/dev/video?"):
            c = cv2.VideoCapture(camera)
            sleep(0.125) # wait for camera to open
            if not(c is None or not c.isOpened()) and len(self.cameras) < 4:
                self.cameras.append(c)
                self.active_cameras.add(f"cam{len(self.cameras)-1}")
                self.captured_cam_events.append(camera)

    def cams(self):
        for i,camera in enumerate(self.active_cameras):
            ret, frame = self.cameras[int(camera[-1])].read()
            if(ret):
                self.string_pub.publish(String(data=  "Hey there's a message"))
                self.camera_publishers[camera].publish(self.br.cv2_to_imgmsg(frame))

    def controlCam(self, cam:int, msg:Bool):
        if(f"cam{cam}" in self.active_cameras and not msg.data):
            self.active_cameras.remove(f"cam{cam}")
        elif(f"cam{cam}" not in self.active_cameras and msg.data):
            self.active_cameras.add(f"cam{cam}")

def sigint_handler(signal, frame):
    rclpy.shutdown()
    exit(0)

def main():
    rclpy.init()
    rclpy.spin(ROVCameras())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
