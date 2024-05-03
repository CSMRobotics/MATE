from common.csm_common_interfaces.msg import PinState
from std_msgs.msg import String
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node

import board
import neopixel

LED_PIN = 0
BLACK = (0,0,0)
WHITE = (255,255,255)
NUM_LIGHTS = 100
PIXELS = neopixel.NeoPixel(LED_PIN, NUM_LIGHTS)

class LEDControllerNode(Node):
    
    def __init__(self):
        super().__init__(node_name="LEDController")
        self.state_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)
        self.rgb_subscriber = self.create_subscription(RGBState, "LED_strip_state", self.on_set_rgb, 10)

        self.publisher_ = self.create_publisher(String, 'preprogrammed_animations', 10)
        
        self.animations = [
            "animation1",
            "animation2"
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
    
    def on_set_rov_gpio(self, message: PinState):
            if message.pin not in LED_PIN:
                pass
            
            PIXELS.fill(WHITE)

    def LED_strip_state(self, message: RGBState):
         pass


def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDControllerNode()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
