from common.csm_common_interfaces.msg import PinState
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node

import board
import neopixel
import time

LED_PIN = 0
BLACK = (0,0,0)
WHITE = (255,255,255)
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
PIXELS = neopixel.NeoPixel(LED_PIN, 100)

class RovGpio(Node):
    
    def __init__(self):
        super().__init__(node_name="LEDController")
        self.state_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)
        self.rgb_subscriber = self.create_subscription(RGBState, "LED_strip_state", self.on_set_rgb, 10)

        GPIO.setmode(GPIO.BOARD)
        
        GPIO.setup(LED_PIN, GPIO.OUT)
        PIXELS.fill(BLACK)

    def on_set_rov_gpio(self, message: PinState):
        if message.pin not in LED_PIN:
            pass
        
        PIXELS.fill(WHITE)

    def LED_strip_state(self, message: RGBState):
        pass
    
    def color_chase(color, wait):
        for i in range(num_pixels):
            PIXELS[i] = color
            time.sleep(wait)
            PIXELS.show()
        time.sleep(0.5)

    def rainbow_cycle(wait):
        for j in range(255):
            for i in range(num_pixels):
                rc_index = (i * 256 // num_pixels) + j
                PIXELS[i] = colorwheel(rc_index & 255)
            PIXELS.show()
            time.sleep(wait)

    def pulse(color, wait):
        for i in range(225):
            

if __name__ == '__main__':
    main()
