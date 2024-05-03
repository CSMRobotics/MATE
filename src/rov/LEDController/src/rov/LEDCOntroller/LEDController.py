from common.csm_common_interfaces.msg import PinState
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node

import board
import neopixel

LED_PIN = 0

class RovGpio(Node):
    def __init__(self):
        super().__init__(node_name="LEDController")
        self.state_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)
        self.rgb_subscriber = self.create_subscription(RGBState, "LED_strip_state", self.on_set_rgb, 10)

    
    GPIO.setmode(GPIO.BOARD)
    pixels = neopixel.NeoPixel(LED_PIN, 100)

    GPIO.setup(LED_PIN, GPIO.OUT)
    
    def on_set_rov_gpio(self, message: PinState):
            if message.pin not in LED_PIN:
                pass

            if message.pin not in self.output_pins:
                GPIO.setup(message.pin, GPIO.OUT)
                self.output_pins.add(message.pin)

            GPIO.output(message.pin, GPIO.HIGH if message.state else GPIO.LOW)

    def LED_strip_state(self, message: RGBState):
         pass
if __name__ == '__main__':
    main()
