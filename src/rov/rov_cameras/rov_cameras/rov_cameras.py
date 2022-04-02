from time import sleep
import cv2
import rclpy
from rclpy.node import Node
import signal

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import glob

FPS = float(30.0)

class ROVCameras(Node):
    def __init__(self):
        super().__init__("rov_cameras")
        self.active_cameras = set()
        self.construct_cameras()
        self.br = CvBridge()

        self.camera_publishers = {
            "cam0":self.create_publisher(Image, "cam0_image", 1),
            "cam1":self.create_publisher(Image, "cam1_image", 1),
            "cam2":self.create_publisher(Image, "cam2_image", 1),
            "cam3":self.create_publisher(Image, "cam3_image", 1)
        }

        self.create_timer(1/FPS, self.cams)
    
    def construct_cameras(self):
        self.cameras = []
        for camera in glob.glob("/dev/video?"):
            c = cv2.VideoCapture(camera)
            sleep(0.125) # wait for camera to open
            if not(c is None or not c.isOpened()) and len(self.cameras) < 4:
                self.cameras.append(c)
                self.active_cameras.add(f"cam{len(self.cameras)-1}")

    def cams(self):
        for camera in self.active_cameras:
            ret, frame = self.cameras[int(camera[-1])].read()
            if(ret):
                self.camera_publishers[camera].publish(self.br.cv2_to_imgmsg(frame))
        pass

    def toggleCam(self, cam:int):
        if(f"cam{cam}" in self.active_cameras):
            self.active_cameras.remove(f"cam{cam}")
        else:
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
