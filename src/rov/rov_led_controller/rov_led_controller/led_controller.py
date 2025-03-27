from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time
import random
from WS2812 import SPItoWS

NUM_LIGHTS = 60
PIXELS = SPItoWS(NUM_LIGHTS)

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
            "off": self.off
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

        # Rainbow RGB values
        self.rainbowR = 255
        self.rainbowG = 255
        self.rainbowB = 0

        # color value and current animation
        self.ledColor = "WHITE"
        self.currentAnimation = "solid"
        
        # Listen for requests from the UI
        self.ui_subscriber = self.create_subscription(String, 'ui_requests', self.ui_request_callback, 10)

        PIXELS.LED_OFF_ALL()
        

    def ui_request_callback(self, msg):
        # Split message
        split = msg.data.split(",")
        msgs = [s.strip() for s in split]

        # Change brightness
        if len(msgs) == 2:
            try:
                brightness = float(msgs[1])
                if brightness > 1.0:
                    brightness = 1
                elif brightness < 0:
                    brightness = 0
                PIXELS.set_brightness(brightness)
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
                # Call the method corresponding to the requested animation
                self.currentAnimation = msgs[0]
                self.get_logger().info('New animation received: %s' % msgs[0])
                self.animations[msgs[0]](self.colors[self.ledColor])
            elif msgs[0] in self.colors:
                # Change the color if message changes colors
                self.ledColor = msgs[0]
                self.get_logger().info('New color received: %s' % msgs[0])
                self.animations[self.currentAnimation](self.colors[msgs[0]])
            

    
    def color_chase(self, color, wait=0.1):
        """
        Animates a single color across the LED strip
        """
        for i in range(NUM_LIGHTS):
            PIXELS.RGBto3Bytes(i, color[0], color[1], color[2])
            time.sleep(wait)
        PIXELS.LED_show()
        time.sleep(0.5)


    def rainbow_cycle(self, color, wait=0.1):
        """
        Cycles a wave of rainbow over the LEDs
        """
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


    def pulse(self, color, wait=0.1):
        """
        Fades a single color in
        """
        for i in range(255):
            for j in range(NUM_LIGHTS):
                PIXELS.RGBto3Bytes(j, color[0] * (i / 255.0), color[1] * (i / 255.0), color[2] * (i / 255.0))
            time.sleep(wait)
            PIXELS.LED_show()
        time.sleep(wait)

    def solid(self, color, wait=0.1):
        """
        Changes all LEDs to solid color
        """
        for i in range (NUM_LIGHTS):
            PIXELS.RGBto3Bytes(i, color[0], color[1], color[2])
        PIXELS.LED_show()

    def seizure_disco(self, color, wait=0.1):
        """
        Changes all LEDs to a random color
        """
        for i in range(NUM_LIGHTS):
            R = random.randint(0,255)
            G = random.randint(0,255)
            B = random.randint(0,255)
            PIXELS.RGBto3Bytes(i, R, G, B)
        PIXELS.LED_show()
        time.sleep(wait)

    def off(self, color):
        PIXELS.LED_OFF_ALL()

    def _set_all_pixels(self, color):
        """
        Sets all pixels to a single
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
