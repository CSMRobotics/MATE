from common.csm_common_interfaces.msg import PinState
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from rainbowio import colorwheel

import board
import neopixel
import time
import random

LED_PIN = 0
BLACK = (0,0,0)
WHITE = (255,255,255)
NUM_LIGHTS = 100
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
PIXELS = neopixel.NeoPixel(LED_PIN, NUM_LIGHTS)
global ledState

class LEDControllerNode(Node):
    
    def __init__(self):
        super().__init__(node_name="LEDController")
        self.state_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)
        self.rgb_subscriber = self.create_subscription(RGBState, "LED_strip_state", self.on_set_rgb, 10)
        self.publisher_ = self.create_publisher(String, 'preprogrammed_animations', 10)
        
        self.animations = [
            "color_chase",
            "rainbow_cycle",
            "pulse",
            "solid",
            "seizure_disco"
        ]
        
        # Listen for requests from the UI
        self.ui_subscriber = self.create_subscription(String, 'ui_requests', self.ui_request_callback, 10)

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LED_PIN, GPIO.OUT)
        PIXELS.fill(BLACK)

    def ui_request_callback(self, msg):
        if msg.data == 'get_animations':
            # Send the list of preprogrammed animations to the UI
            animations_msg = String()
            animations_msg.data = "\n".join(animation for animation in self.animations if animation.startswith('custom_'))
            self.publisher_.publish(animations_msg)
            self.get_logger().info('Sent preprogrammed animations to UI')
        else:
            # Check if the requested animation exists
            if msg.data in self.animations:
                # Call the method corresponding to the requested animation
                
                if ledState and (lastAnimation != msg.data):
                    getattr(self, lastAnimation)()
                else:
                    PIXELS.brightness(0)
            lastAnimation = msg.data

    def on_set_rov_gpio(self, message: PinState):
        if message.pin not in LED_PIN:
            pass
        ledState = message.state

def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDControllerNode()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()
    
    
    def color_chase(color, wait):
        for i in range(NUM_LIGHTS):
            PIXELS[i] = color
            time.sleep(wait)
            PIXELS.show()
        time.sleep(0.5)

    def rainbow_cycle(wait):
        for j in range(255):
            for i in range(NUM_LIGHTS):
                rc_index = (i * 256 // NUM_LIGHTS) + j
                PIXELS[i] = colorwheel(rc_index & 255)
            PIXELS.show()
            time.sleep(wait)

    def pulse(color, wait):
        PIXELS.fill(color)
        for i in range(255):
            PIXELS.setBrightness(i)
            time.sleep(wait)
            PIXELS.show()
        time.sleep(wait)

    def solid(color):
        PIXELS.fill(color)
        PIXELS.show()

    def seizure_disco(wait):
        for i in range(NUM_LIGHTS):
            R = random.randint(0,255)
            G = random.randint(0,255)
            B = random.randint(0,255)
            PIXELS[i].fill(R, G, B)
        PIXELS.show()
        time.sleep(wait)


def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDControllerNode()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
