from time import sleep
from ament_index_python import get_package_share_directory
import rclpy
import signal
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from sensor_msgs.msg import CompressedImage, Joy
import threading
from flask import Flask, request, render_template, render_template_string, Response
import cv2
from cv_bridge import CvBridge
import os
import json

from rov_interfaces.msg import JetsonNanoStatistics, BNO055Data

FPS = float(30.0)
PREFIX = get_package_share_directory("bs_flask")
DEFAULT_IMAGE = cv2.imread(os.path.join(PREFIX,"default.jpg"))
FROZEN_IMAGE = cv2.imread(os.path.join(PREFIX,"frozen.jpg"))

def convertNumpyArrayToHTMLTag(ndarray):
    _, buffer = cv2.imencode('.jpg', ndarray) #TODO: this can be simplified?? compressed image msg is already sent as a JPG....save time maybe??
    return buffer.tobytes() # readable by html-img tag

class Flask_Node(Node):
    def __init__(self) -> None:
        super().__init__("flask_node")
        self.br = CvBridge()

        # subscriptions to handle receiving images
        self.camera_subscriptions = {
            "cam0":self.create_subscription(CompressedImage, 'cam0_image', self.camera0_callback, 10),
            "cam1":self.create_subscription(CompressedImage, 'cam1_image', self.camera1_callback, 10),
            "cam2":self.create_subscription(CompressedImage, 'cam2_image', self.camera2_callback, 10),
            "cam3":self.create_subscription(CompressedImage, 'cam3_image', self.camera3_callback, 10)
        }

        self.test_sub = self.create_subscription(String, "test", self.test, 10)

        # publishers to control activation of cameras
        self.camera_control = {
            "cam0":self.create_publisher(Bool, "cam0_control", 10),
            "cam1":self.create_publisher(Bool, "cam1_control", 10),
            "cam2":self.create_publisher(Bool, "cam2_control", 10),
            "cam3":self.create_publisher(Bool, "cam3_control", 10)
        }

        tmp = convertNumpyArrayToHTMLTag(DEFAULT_IMAGE)
        self.htmltags = [tmp, tmp, tmp, tmp]
        self.image_dirty = [False, False, False, False]
        self.html_locks = [threading.Lock(), threading.Lock(), threading.Lock(), threading.Lock()]

        self.rov_statistics_subscriber = self.create_subscription(JetsonNanoStatistics, "rov_statistics", self.rov_statistics_callback, 2)
        self.rov_statistics = JetsonNanoStatistics()
        self.rov_statistics_lock = threading.Lock()

        self.joystick_subscriber = self.create_subscription(Joy, "joy", self.joystick_callback, 10)
        self.joy_data = Joy()

        self.bno_subscriber = self.create_subscription(BNO055Data, "bno055_data", self.bno_callback, qos_profile_sensor_data)
        self.bno_data = BNO055Data()

        self.estop_publisher = self.create_publisher(Bool, "estop", 10)
        # self.are_we_frozen = self.create_timer(3, self.freeze_catcher)

    def test(self, msg):
        print(msg.data)

    def bno_callback(self, msg:BNO055Data):
        self.bno_data = msg

    def get_attitude(self):
        while True:
            sleep(1/120)
            yield "data:" + json.dumps({
                "orientation": {
                    "w": self.bno_data.orientation.w,
                    "x": self.bno_data.orientation.i,
                    "y": self.bno_data.orientation.j,
                    "z": self.bno_data.orientation.k
                },
                "gravity": {
                    "x": self.bno_data.gravity.i,
                    "y": self.bno_data.gravity.j,
                    "z": self.bno_data.gravity.k
                },
                "magnetometer": {
                    "x": self.bno_data.magnetometer.i,
                    "y": self.bno_data.magnetometer.j,
                    "z": self.bno_data.magnetometer.k
                }
            }) + "\n\n"

    def joystick_callback(self, msg:Joy):
        # do something with joystick data
        self.joy_data = msg

    def get_joystick(self):
        while True:
            sleep(1/120)
            yield "data:" + json.dumps({
                "axes": list(self.joy_data.axes),
                "buttons": list(self.joy_data.buttons)
            }) + "\n\n"

    # ouch my eyes
    def rov_statistics_callback(self, msg:JetsonNanoStatistics):
        self.rov_statistics_lock.acquire()
        self.rov_statistics = msg
        self.rov_statistics_lock.release()

    def freeze_catcher(self):
        for i in range(0,4):
            if(self.image_dirty[i] != True):
                # image has frozen, replace it with frozen image
                if(self.htmltags[i] != convertNumpyArrayToHTMLTag(DEFAULT_IMAGE)):
                    self.htmltags[i] = convertNumpyArrayToHTMLTag(FROZEN_IMAGE)
            else:
                self.image_dirty[i] = False

    def camera_onoff(self, index:int, onoff:bool):
        msg = Bool()
        msg.data=onoff
        self.camera_control[index].publish(msg)

    def camera0_callback(self, msg:CompressedImage):
        self.html_locks[0].acquire()
        self.htmltags[0] = convertNumpyArrayToHTMLTag(self.br.compressed_imgmsg_to_cv2(msg))
        self.image_dirty[0] = True
        self.html_locks[0].release()

    def camera1_callback(self, msg:CompressedImage):
        self.html_locks[1].acquire()
        self.htmltags[1] = convertNumpyArrayToHTMLTag(self.br.compressed_imgmsg_to_cv2(msg))
        self.image_dirty[1] = True
        self.html_locks[1].release()

    def camera2_callback(self, msg:CompressedImage):
        self.html_locks[2].acquire()
        self.htmltags[2] = convertNumpyArrayToHTMLTag(self.br.compressed_imgmsg_to_cv2(msg))
        self.image_dirty[2] = True
        self.html_locks[2].release()

    def camera3_callback(self, msg:CompressedImage):
        self.html_locks[3].acquire()
        self.htmltags[3] = convertNumpyArrayToHTMLTag(self.br.compressed_imgmsg_to_cv2(msg))
        self.image_dirty[3] = True
        self.html_locks[3].release()

    def getImage(self, index):
        while True:
            sleep(1/FPS)
            self.html_locks[index].acquire()
            htmltag = self.htmltags[index]
            self.html_locks[index].release()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + htmltag + b'\r\n\r\n')
    
    def estop(self):
        msg = Bool()
        msg.data = True
        self.estop_publisher.publish(msg)

def ros2_thread(node):
    rclpy.spin(node)

def sigint_handler(signal, frame):
    """
    SIGINT handler
    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)

app = Flask(PREFIX,template_folder=PREFIX+"/templates",static_folder=PREFIX+"/static")
prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)

@app.route('/')
def index():
    """Video streaming"""
    return render_template("index.html")

@app.route('/camera_1_video_feed')
def camera_1_video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(ros2_node.getImage(0),
                mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera_2_video_feed')
def camera_2_video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(ros2_node.getImage(1),
                mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera_3_video_feed')
def camera_3_video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(ros2_node.getImage(2),
                mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/camera_4_video_feed')
def camera_4_video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(ros2_node.getImage(3),
                mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/shell")
def shellCommand():
    recieved_command = request.args.get("command")
    print("Recieved command from JS: {}".format(recieved_command))

    try:
        recieved_command_result = eval(recieved_command)
    except Exception as e:
        recieved_command_result = e

    print("Sending command result to JS: {}".format(recieved_command_result))
    return {"result": recieved_command_result}

@app.route("/get_joystick")
def get_joystick():
    return Response(ros2_node.get_joystick(), mimetype="text/event-stream")

@app.route("/get_attitude")
def get_attitude():
    return Response(ros2_node.get_attitude(), mimetype="text/event-stream")

def main(args=None):
    global ros2_node
    rclpy.init(args=args)
    ros2_node = Flask_Node()
    threading.Thread(target=ros2_thread, args=[ros2_node]).start()
    app.run()

if __name__ == "__main__":
    main()
