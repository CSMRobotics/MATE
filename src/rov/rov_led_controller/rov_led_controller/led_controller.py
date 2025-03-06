from common.csm_common_interfaces.msg import PinState
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node

import time
import random

from WS2812 import SPItoWS

NUM_LIGHTS = 60
BLACK = (0,0,0)
WHITE = (255,255,255)
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
PIXELS = SPItoWS(NUM_LIGHTS)

global ledColor

class LEDControllerNode(Node):
    
    def __init__(self):
        super().__init__(node_name="LEDController")
        self.state_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)
        self.publisher_ = self.create_publisher(String, 'preprogrammed_animations', 10)
        
        self.animations = {
            "color_chase": self.color_chase, 
            "rainbow_cycle": self.rainbow_cycle,
            "pulse": self.pulse,
            "solid": self.solid,
            "seizure_disco": self.seizure_disco
        }

        self.colors = [
            "WHITE",
            "BLACK",
            "RED",
            "YELLOW",
            "GREEN",
            "CYAN",
            "BLUE",
            "PURPLE"
        ]

        # Rainbow RGB values
        self.rainbowR = 255
        self.rainbowG = 255
        self.rainbowB = 0
        
        # Listen for requests from the UI
        self.ui_subscriber = self.create_subscription(String, 'ui_requests', self.ui_request_callback, 10)

        PIXELS.LED_OFF_ALL()
        

    def ui_request_callback(self, msg):
        if msg.data == 'get_animations':
            # Send the list of preprogrammed animations to the UI
            animations_msg = String()
            animations_msg.data = "\n".join(animation for animation in self.animations)
            self.publisher_.publish(animations_msg)
            self.get_logger().info('Sent preprogrammed animations to UI')
        elif msg.data == 'get_colors':
            colors_msg = String()
            colors_msg.data = "\n".join(color for color in self.colors)
            self.publisher_.publish(colors_msg)
            self.get_logger().info('Sent preprogrammed colors to UI')
        else:
            # Check if the requested animation exists
            if msg.data in self.animations:
                # Call the method corresponding to the requested animation
                self.animations[msg.data]()

    
    def color_chase(color, wait=0.1):
        for i in range(NUM_LIGHTS):
            PIXELS.RGBto3Bytes(i, color[0], color[1], color[2])
            time.sleep(wait)
        PIXELS.LED_show()
        time.sleep(0.5)


    def rainbow_cycle(self, color, wait=0.1):
        print("Hello world")
        RGBincrement = float(255 * 3.0) / NUM_LIGHTS
        # LED RGB values
        tempR = self.rainbowR
        tempG = self.rainbowG
        tempB = self.rainbowB

        # Assigning LED RGB values
        for i in range(NUM_LIGHTS):
            if tempR == 255 and tempB == 0 and tempG < 255:
                PIXELS.RGBto3Bytes(i, tempR, tempG, tempB)
                tempG += RGBincrement
            elif tempG == 255 and tempB == 0 and tempR > 0:
                PIXELS.RGBto3Bytes(i, tempR, tempG, tempB)
                tempR -= RGBincrement
            elif tempG == 255 and tempR == 0 and tempB < 255:
                PIXELS.RGBto3Bytes(i, tempR, tempG, tempB)
                tempB += RGBincrement
            elif tempB == 255 and tempR == 0 and tempG > 0:
                PIXELS.RGBto3Bytes(i, tempR, tempG, tempB)
                tempG -= RGBincrement
            elif tempB == 255 and tempG == 0 and tempR < 255:
                PIXELS.RGBto3Bytes(i, tempR, tempG, tempB)
                tempR += RGBincrement
            elif tempR == 255 and tempG == 0 and tempB > 0:
                PIXELS.RGBto3Bytes(i, tempR, tempG, tempB)
                tempB -= RGBincrement
        PIXELS.LED_show()
        time.sleep(wait)

        # increment first rainbow RGB values to next value
        if self.rainbowR == 255 and self.rainbowB == 0 and self.rainbowG < 255:
            self.rainbowG += 1
        elif self.rainbowG == 255 and self.rainbowB == 0 and self.rainbowR > 0:
            self.rainbowR -= 1
        elif self.rainbowG == 255 and self.rainbowR == 0 and self.rainbowB < 255:
            self.rainbowB += 1
        elif self.rainbowB == 255 and self.rainbowR == 0 and self.rainbowG > 0:
            self.rainbowG -= 1
        elif self.rainbowB == 255 and self.rainbowG == 0 and self.rainbowR < 255:
            self.rainbowR += 1
        elif self.rainbowR == 255 and self.rainbowG == 0 and self.rainbowB > 0:
            self.rainbowB -= 1


    def pulse(color, wait=0.1):
        for i in range(255):
            for j in range(NUM_LIGHTS):
                PIXELS.RGBto3Bytes(j, color[0] * (i / 255.0), color[1] * (i / 255.0), color[2] * (i / 255.0))
            time.sleep(wait)
            PIXELS.LED_show()
        time.sleep(wait)

    def solid(color, wait=0.1):
        for i in range (NUM_LIGHTS):
            PIXELS.RGBto3Bytes(i, color[0], color[1], color[2])
        PIXELS.LED_show()

    def seizure_disco(color, wait=0.1):
        for i in range(NUM_LIGHTS):
            R = random.randint(0,255)
            G = random.randint(0,255)
            B = random.randint(0,255)
            PIXELS.RGBto3Bytes(i, R, G, B)
        PIXELS.LED_show()
        time.sleep(wait)

    def _set_all_pixels(color):
        """
        Sets all pixels to a single
        :param color: color to set all pixels to
        """
        for i in range (NUM_LIGHTS):
            PIXELS.RGBto3Bytes(i, color[0], color[1], color[2])



def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDControllerNode()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
