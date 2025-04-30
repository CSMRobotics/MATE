from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time
import random
import colorsys
import threading
from rov_led_controller.WS2812 import SPItoWS
from rov_led_controller.SK6812RGBW import SPItoSK


NUM_LEDS = 60
RGBW_PIXELS = SPItoSK(NUM_LEDS) 

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
        self.stopLock = threading.Lock()

        RGBW_PIXELS.LED_OFF_ALL()
        

    def ui_request_callback(self, msg):
        # Split message
        split = msg.data.split(",")
        msgs = [s.strip() for s in split]

        # Change brightness
        if len(msgs) == 2:
            try:
                # set and clamp brightness between 0 and 1
                brightness = max(0.0, min(float(msgs[1]), 1.0))
                RGBW_PIXELS.set_brightness(brightness * 0.5)
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
                with self.stopLock:
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
                with self.stopLock:
                    self.stopAnim = True
                while threading.active_count() > 1:
                    time.sleep(0.1)
                self.stopAnim = False
                # Change the color if message changes colors
                self.ledColor = msgs[0]
                self.get_logger().info('New color received: %s' % msgs[0])
                tempThread = threading.Thread(target=self.animations[self.currentAnimation], args=(self.colors[msgs[0]],))
                tempThread.start()
            

    
    def color_chase(self, color, wait=1/NUM_LEDS):
        """
        Animates a single color across the LED strip
        """
        while self.stopAnim == False:
            for i in range(NUM_LEDS):
                self._set_RGBW_LED(i, color[0], color[1], color[2])
                time.sleep(wait)
                RGBW_PIXELS.LED_show()
                if self.stopAnim == True:
                    break
            for i in range(NUM_LEDS):
                self._set_RGBW_LED(i, 0, 0, 0)
                time.sleep(wait)
                RGBW_PIXELS.LED_show()
                if self.stopAnim == True:
                    break


    def rainbow_cycle(self, color, wait = 2 / NUM_LEDS):
        """
        Cycles a wave of rainbow over the LEDs
        """
        while self.stopAnim == False:
            for i in range(256):
                for j in range (NUM_LEDS):
                    R = self.rainbowValues[(j + i) % 256][0]
                    G = self.rainbowValues[(j + i) % 256][1]
                    B = self.rainbowValues[(j + i) % 256][2]
                    self._set_RGBW_LED(j, R, G, B)
                RGBW_PIXELS.LED_show()
                time.sleep(wait)
                if self.stopAnim == True:
                    break


    def pulse(self, color, wait=0.005):
        """
        Fades a single color in and out
        """
        while self.stopAnim == False:
            for i in range(255):
                R = color[0] * (i / 255.0)
                G = color[1] * (i / 255.0)
                B = color[2] * (i / 255.0)
                self._set_all_pixels((R, G, B))
                time.sleep(wait)
                RGBW_PIXELS.LED_show()
                if self.stopAnim == True:
                    break
            for i in range(255):
                R = color[0] * ((255 - i) / 255.0)
                G = color[1] * ((255 - i) / 255.0)
                B = color[2] * ((255 - i) / 255.0)
                self._set_all_pixels((R, G, B))
                time.sleep(wait)
                RGBW_PIXELS.LED_show()
                if self.stopAnim == True:
                    break


    def solid(self, color, wait=0.1):
        """
        Changes all LEDs to solid color
        """
        self._set_all_pixels((color[0], color[1], color[2]))
        RGBW_PIXELS.LED_show()


    def seizure_disco(self, color, wait=0.1):
        """
        Changes all LEDs to a random color
        """
        for i in range(NUM_LEDS):
            R = random.randint(0,255)
            G = random.randint(0,255)
            B = random.randint(0,255)
            self._set_RGBW_LED(i, R, G, B)
        RGBW_PIXELS.LED_show()
        time.sleep(wait)

    def off(self, color):
        RGBW_PIXELS.LED_OFF_ALL()

    def boot(self, color):
        """
        Booting animation
        """
        for i in range(NUM_LEDS):
            self._set_RGBW_LED(i, 0, 255, 0)
            RGBW_PIXELS.LED_show()
            time.sleep(1/NUM_LEDS)
        time.sleep(0.3)
        time.sleep(0.3)
        self._set_all_pixels(self.colors["GREEN"])
        time.sleep(1)
        

    def _set_all_pixels(self, color):
        """
        Sets all pixels to a single
        """
        for i in range (NUM_LEDS):
            self._set_RGBW_LED(i, color[0], color[1], color[2])
    

    def _set_RGBW_LED(self, ledNum, R, G, B):
        """
        Convert RGB value to RGBW value and set RGBW LEDs to new color
        """
        # percentage of RGB in old colorspace
        RGBTotal = R + G + B
        if RGBTotal == 0:
             RGBW_PIXELS.set_LED_color(ledNum, 0, 0, 0, 0)
             return
        percentR = R/RGBTotal
        percentG = G/RGBTotal
        percentB = B/RGBTotal
        # set white value as lowest RGB value
        newW = min(R, G, B)
        # To preserve max brightness, set new highest RGB value to corresponding RGBW value
        # Then scale the value of the 2 other RGB values to preserve original color percentage
        newR = 0
        newG = 0
        newB = 0
        if max(R, G, B) == R:
            newR = R
            RTotal = newR + newW
            RGBWTotal = RTotal / percentR
            newG = (RGBWTotal * percentG) - newW
            newB = (RGBWTotal * percentB) - newW
        elif max(R, G, B) == G:
            newG = G
            GTotal = newG + newW
            RGBWTotal = GTotal / percentG
            newR = (RGBWTotal * percentR) - newW
            newB = (RGBWTotal * percentB) - newW
        elif max(R, G, B) == B:
            newB = B
            BTotal = newB + newW
            RGBWTotal = BTotal / percentB                
            newR = (RGBWTotal * percentR) - newW
            newG = (RGBWTotal * percentG) - newW
        RGBW_PIXELS.set_LED_color(ledNum, newR, newG, newB, newW)
        
        


def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDControllerNode()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
