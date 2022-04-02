from time import sleep
from ament_index_python import get_package_share_directory
import rclpy
import signal
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import threading
from flask import Flask, request, render_template, render_template_string, Response
from cv_bridge import CvBridge
import cv2
import os

FPS = float(30.0)
PREFIX = get_package_share_directory("bs_flask")
DEFAULT_IMAGE = cv2.imread(os.path.join(PREFIX,"default.jpg"))

class Flask_Node(Node):
    def __init__(self) -> None:
        super().__init__("flask_node")
        self.br = CvBridge()

        # subscriptions to handle receiving images
        self.camera_subscriptions = {
            "cam0":self.create_subscription(Image, 'cam0_image', self.camera0_callback, 1),
            "cam1":self.create_subscription(Image, 'cam1_image', self.camera1_callback, 1),
            "cam2":self.create_subscription(Image, 'cam2_image', self.camera2_callback, 1),
            "cam3":self.create_subscription(Image, 'cam3_image', self.camera3_callback, 1)
        }

        # publishers to control activation of cameras
        self.camera_control = {
            "cam0":self.create_publisher(Bool, "cam0_control", 10),
            "cam1":self.create_publisher(Bool, "cam1_control", 10),
            "cam2":self.create_publisher(Bool, "cam2_control", 10),
            "cam3":self.create_publisher(Bool, "cam3_control", 10)
        }

        self.images = [DEFAULT_IMAGE, DEFAULT_IMAGE, DEFAULT_IMAGE, DEFAULT_IMAGE]
        self.locks = [threading.Lock(), threading.Lock(), threading.Lock(), threading.Lock()]
    
    def freeze_catcher():
        pass

    def camera0_callback(self, msg:Image):
        self.locks[0].acquire()
        self.images[0] = self.br.imgmsg_to_cv2(msg)
        self.locks[0].release()

    def camera1_callback(self, msg:Image):
        self.locks[1].acquire()
        self.images[1] = self.br.imgmsg_to_cv2(msg)
        self.locks[1].release()

    def camera2_callback(self, msg:Image):
        self.locks[2].acquire()
        self.images[2] = self.br.imgmsg_to_cv2(msg)
        self.locks[2].release()

    def camera3_callback(self, msg:Image):
        self.locks[3].acquire()
        self.images[3] = self.br.imgmsg_to_cv2(msg)
        self.locks[3].release()

    def convertNumpyArrayToHTMLTag(self, ndarray):
        _, buffer = cv2.imencode('.jpg', ndarray)
        return buffer.tobytes() # readable by html-img tag

    def getImage(self, index):
        while True:
            sleep(1/FPS)
            self.locks[index].acquire()
            htmltag = self.convertNumpyArrayToHTMLTag(self.images[index])
            self.locks[index].release()
            yield (b'--frame\r\n'
                b'Content-Type: image/jpeg\r\n\r\n' + htmltag + b'\r\n\r\n')

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

def main(args=None):
    global ros2_node
    rclpy.init(args=args)
    ros2_node = Flask_Node()
    threading.Thread(target=ros2_thread, args=[ros2_node]).start()
    app.run()

if __name__ == "__main__":
    main()