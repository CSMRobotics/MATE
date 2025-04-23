from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time
import random
import colorsys
import threading
from rov_led_controller.WS2812 import SPItoWS
from rov_led_controller.SK6812RGBW import SPItoSK


NUM_LIGHTS = 60
RGB_PIXELS = SPItoWS(NUM_LIGHTS)
RGBW_PIXELS = SPItoSK(NUM_LIGHTS) 

class LEDControllerNode(Node):
    
    def __init__(self):
        super().__init__(node_name="LEDController")
        self.publisher_ = self.create_publisher(String, 'led_controller_output', 10)
        
        self.animations = {
            "color_chase": self.color_chase, 
            "rainbow_cycle": self.rainbow_cycle,
            "pulse": self.pulse,
            "solid": self.solid,
            "seizure_disco": self.seizure_disco,
            "off": self.off,
            "boot": self.boot
        }

        self.colors = {
            "WHITE": (255,255,255),
            "BLACK": (0,0,0),
            "RED": (255, 0, 0),
            "YELLOW": (255, 150, 0),
            "GREEN": (0, 255, 0),
            "CYAN": (0, 255, 255),
            "BLUE": (0, 0, 255),
            "PURPLE": (180, 0, 255)
        }

        # Calculate rainbow RGB values
        self.rainbowValues = []
        hueIncrement = 1.0 / 256
        for i in range (1, 257):
            rgb = colorsys.hsv_to_rgb(i * hueIncrement, 1, 1)
            scaled_rgb = tuple(int(value * 255) for value in rgb)
            self.rainbowValues.append(scaled_rgb)        

        # color value and current animation
        self.ledColor = "WHITE"
        self.currentAnimation = "solid"
        
        # Listen for requests from the UI
        self.ui_subscriber = self.create_subscription(String, 'ui_requests', self.ui_request_callback, 10)

        # Stop flag
        self.stopAnim = False

        RGB_PIXELS.LED_OFF_ALL()
        RGBW_PIXELS.set_LED_color(40, 255, 255, 255, 255)
        

    def ui_request_callback(self, msg):
        # Split message
        split = msg.data.split(",")
        msgs = [s.strip() for s in split]

        # Change brightness
        if len(msgs) == 2:
            try:
                # set and clamp brightness between 0 and 1
                brightness = max(0.0, min(float(msgs[1]), 1.0))
                RGB_PIXELS.set_brightness(brightness * 0.5)
                self.get_logger().info("Changed brightness to: %f" % brightness)
            except:
                self.get_logger().info("Second argument, brightness, should be a float.")
        
        # Change animation and colors
        if msgs[0] == 'get_animations':
            # Send the list of preprogrammed animations to the UI
            animations_msg = String()
            animations_msg.data = "\n".join(animation for animation in self.animations)
            self.publisher_.publish(animations_msg)
            self.get_logger().info('Sent preprogrammed animations to UI')
        elif msgs[0] == 'get_colors':
            colors_msg = String()
            colors_msg.data = "\n".join(color for color in self.colors)
            self.publisher_.publish(colors_msg)
            self.get_logger().info('Sent preprogrammed colors to UI')
        else:
            # Check if the requested animation exists
            if msgs[0] in self.animations:                    
                # Stop current animation
                self.stopAnim = True
                while threading.active_count() > 1:
                    time.sleep(0.1)
                self.stopAnim = False
                # Call the method corresponding to the requested animation
                self.currentAnimation = msgs[0]
                self.get_logger().info('New animation received: %s' % msgs[0])
                tempThread = threading.Thread(target=self.animations[msgs[0]], args=(self.colors[self.ledColor],))
                tempThread.start()
            elif msgs[0] in self.colors:
                # Stop current animation
                self.stopAnim = True
                while threading.active_count() > 1:
                    time.sleep(0.1)
                self.stopAnim = False
                # Change the color if message changes colors
                self.ledColor = msgs[0]
                self.get_logger().info('New color received: %s' % msgs[0])
                self.animations[self.currentAnimation](self.colors[msgs[0]])
            

    
    def color_chase(self, color, wait=1/NUM_LIGHTS):
        """
        Animates a single color across the LED strip
        """
        RGB_PIXELS.LED_OFF_ALL()
        while self.stopAnim == False:
            for i in range(NUM_LIGHTS):
                RGB_PIXELS.set_LED_color(i, color[0], color[1], color[2])
                time.sleep(wait)
                RGB_PIXELS.LED_show()
                if self.stopAnim == True:
                    break
            for i in range(NUM_LIGHTS):
                RGB_PIXELS.set_LED_color(i, 0, 0, 0)
                time.sleep(wait)
                RGB_PIXELS.LED_show()
                if self.stopAnim == True:
                    break


    def rainbow_cycle(self, color, wait = 2 / NUM_LIGHTS):
        """
        Cycles a wave of rainbow over the LEDs
        """
        while self.stopAnim == False:
            for i in range(256):
                for j in range (NUM_LIGHTS):
                    RGB_PIXELS.set_LED_color(j, self.rainbowValues[(j + i) % 256][0], self.rainbowValues[(j + i) % 256][1], self.rainbowValues[(j + i) % 256][2])
                RGB_PIXELS.LED_show()
                time.sleep(wait)
                if self.stopAnim == True:
                    break


    def pulse(self, color, wait=0.1):
        """
        Fades a single color in and out
        """
        while self.stopAnim == False:
            for i in range(255):
                for j in range(NUM_LIGHTS):
                    RGB_PIXELS.set_LED_color(j, color[0] * (i / 255.0), color[1] * (i / 255.0), color[2] * (i / 255.0))
                time.sleep(wait)
                RGB_PIXELS.LED_show()
                if self.stopAnim == True:
                    break


    def solid(self, color, wait=0.1):
        """
        Changes all LEDs to solid color
        """
        for i in range (NUM_LIGHTS):
            RGB_PIXELS.set_LED_color(i, color[0], color[1], color[2])
        RGB_PIXELS.LED_show()


    def seizure_disco(self, color, wait=0.1):
        """
        Changes all LEDs to a random color
        """
        for i in range(NUM_LIGHTS):
            R = random.randint(0,255)
            G = random.randint(0,255)
            B = random.randint(0,255)
            RGB_PIXELS.set_LED_color(i, R, G, B)
        RGB_PIXELS.LED_show()
        time.sleep(wait)

    def off(self, color):
        RGB_PIXELS.LED_OFF_ALL()

    def boot(self, color):
        """
        Booting animation
        """
        for i in range(NUM_LIGHTS):
            RGB_PIXELS.set_LED_color(i, 0, 255, 0)
            RGB_PIXELS.LED_show()
            time.sleep(1/NUM_LIGHTS)
        time.sleep(0.3)
        RGB_PIXELS.LED_OFF_ALL()
        time.sleep(0.3)
        self._set_all_pixels(self.colors["GREEN"])
        time.sleep(1)
        RGB_PIXELS.LED_OFF_ALL()
        

    def _set_all_pixels(self, color):
        """
        Sets all pixels to a single
        """
        for i in range (NUM_LIGHTS):
            RGB_PIXELS.set_LED_color(i, color[0], color[1], color[2])

        


def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDControllerNode()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
