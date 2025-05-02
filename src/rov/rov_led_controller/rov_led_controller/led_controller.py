from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import time
import random
import colorsys
import threading
from rov_led_controller.SK6812RGBW import SPItoSK


# Ring leds go first, then strip leds
NUM_RING_LEDS = 24
NUM_STRIP_LEDS = 48
RGBW_PIXELS = SPItoSK(NUM_RING_LEDS + NUM_STRIP_LEDS)

class LEDControllerNode(Node):
    
    def __init__(self):
        super().__init__(node_name="LEDController")
        
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
        self.ringColor = "WHITE"
        self.ringCurrAnim = "off"
        self.stripColor = "WHITE"
        self.stripCurrAnim = "off"
        
        # Listen for requests from the UI
        self.ui_subscriber = self.create_subscription(String, 'ui_requests', self.ui_request_callback, 10)

        # SK6812 Mutex
        self.LEDMutex = threading.Lock()

        # Thread management
        # Can't believe I need to wrap StopAnim in a list to pass by reference
        self.ringStopAnim = [False]
        self.stripStopAnim = [False]
        self.stripStopMutex = threading.Lock()
        
        # Set leds to off when starting
        self.ringAnimThread = threading.Thread(target=self.animations[self.ringCurrAnim], args=(self.colors[self.ringColor], self._set_ring_LED))
        self.stripAnimThread = threading.Thread(target=self.animations[self.stripCurrAnim], args=(self.colors[self.stripColor], self._set_strip_LED))
        self.ringAnimThread.start()
        self.stripAnimThread.start()

    def ui_request_callback(self, msg):
        # Split message
        split = msg.data.split(",")
        msgs = [s.strip() for s in split]
        
        # Change brightness
        if len(msgs) == 3:
            try:
                # set and clamp brightness between 0 and 1
                brightness = max(0.0, min(float(msgs[2]), 1.0))
                RGBW_PIXELS.set_brightness(brightness)
                self.get_logger().info("Changed brightness to: %f" % brightness)
            except:
                self.get_logger().info("Third argument, brightness, should be a number from 0-1.")
        else:
            RGBW_PIXELS.set_brightness(0.25)
        
        # Change animation and colors
        if msgs[0] == 'get_animations':
            # Send the list of preprogrammed animations to the UI
            animations_msg = String()
            animations_msg.data = "\n".join(animation for animation in self.animations)
            self.get_logger().info(animations_msg)
        elif msgs[0] == 'get_colors':
            colors_msg = String()
            colors_msg.data = "\n".join(color for color in self.colors)
            self.get_logger().info(colors_msg)
        elif len(msgs) >= 2:            
            # Check for which leds the user wants to modify
            setStripFunc = self._set_ring_LED
            if msgs[0] == "strip":
                setStripFunc = self._set_strip_LED
            elif msgs[0] != "ring":
                self.get_logger().info("First argument must be \"ring\" or \"strip\", defaulting to ring")
                msgs[0] = "ring"
            # Check if the requested animation exists
            if msgs[1] in self.animations:                    
                # Stop current animation
                if msgs[0] == "ring":
                    self.ringStopAnim[0] = True
                    self.ringAnimThread.join()
                    self.ringStopAnim[0] = False
                elif msgs[0] == "strip":
                    with self.stripStopMutex:
                        self.stripStopAnim[0] = True
                    self.stripAnimThread.join()
                    self.stripStopAnim[0] = False
                # Call the method corresponding to the requested animation
                if msgs[0] == "ring":
                    self.ringCurrAnim = msgs[1]
                    self.get_logger().info('New ring animation received: %s' % msgs[1])
                    self.ringAnimThread = threading.Thread(target=self.animations[msgs[1]], args=(self.colors[self.ringColor], setStripFunc))
                    self.ringAnimThread.start()
                elif msgs[0] == "strip":
                    self.stripCurrAnim = msgs[1]
                    self.get_logger().info('New strip animation received: %s' % msgs[1])
                    self.stripAnimThread = threading.Thread(target=self.animations[msgs[1]], args=(self.colors[self.stripColor], setStripFunc))
                    self.stripAnimThread.start()
            elif msgs[1] in self.colors:
                # Stop current animation
                if msgs[0] == "ring":
                    self.ringStopAnim[0] = True
                    self.ringAnimThread.join()
                    self.ringStopAnim[0] = False
                elif msgs[0] == "strip":
                    with self.stripStopMutex:
                        self.stripStopAnim[0] = True
                    self.stripAnimThread.join()
                    self.stripStopAnim[0] = False
                # Change the color if message changes colors
                if msgs[0] == "ring":
                    self.ringColor = msgs[1]
                    self.get_logger().info('New ring color received: %s' % msgs[1])
                    self.ringAnimThread = threading.Thread(target=self.animations[self.ringCurrAnim], args=(self.colors[self.ringColor], setStripFunc))
                    self.ringAnimThread.start()
                elif msgs[0] == "strip":
                    self.stripColor = msgs[1]
                    self.get_logger().info('New strip color received: %s' % msgs[1])
                    self.stripAnimThread = threading.Thread(target=self.animations[self.stripCurrAnim], args=(self.colors[self.stripColor], setStripFunc))
                    self.stripAnimThread.start()
            else:
                self.get_logger().info("Second argument must be an animation or color. Send \"get_animations\" or \"get_colors\" to view options.")
        else:
            self.get_logger().info("Request most be formatted: \"ring\"/\"strip\", animation/color, [brightness]. Send \"get_animations\" or \"get_colors\" to view options.")
            

    
    def color_chase(self, color, setStripFunc):
        """
        Animates a single color across the LED strip
        """
        numLEDs = 0
        if setStripFunc == self._set_ring_LED:
            numLEDs = NUM_RING_LEDS
            stopAnim = self.ringStopAnim
        else:
            numLEDs = NUM_STRIP_LEDS
            stopAnim = self.stripStopAnim
        wait = 1/numLEDs
        while stopAnim == [False]:
            for i in range(numLEDs):
                setStripFunc(i, color[0], color[1], color[2])
                time.sleep(wait)
                self._LED_show()
                if stopAnim == [True]:
                    break
            for i in range(numLEDs):
                setStripFunc(i, 0, 0, 0)
                time.sleep(wait)
                self._LED_show()
                if stopAnim == [True]:
                    break


    def rainbow_cycle(self, color, setStripFunc):
        """
        Cycles a wave of rainbow over the LEDs
        """
        numLEDs = 0
        if setStripFunc == self._set_ring_LED:
            numLEDs = NUM_RING_LEDS
            stopAnim = self.ringStopAnim
        else:
            numLEDs = NUM_STRIP_LEDS
            stopAnim = self.stripStopAnim
        wait = 0.05
        while stopAnim == [False]:
            for i in range(256):
                for j in range (numLEDs):
                    R = self.rainbowValues[(j + i) % 256][0]
                    G = self.rainbowValues[(j + i) % 256][1]
                    B = self.rainbowValues[(j + i) % 256][2]
                    setStripFunc(j, R, G, B)
                self._LED_show()
                time.sleep(wait)
                if stopAnim == [True]:
                    break


    def pulse(self, color, setStripFunc):
        """
        Fades a single color in and out
        """
        if setStripFunc == self._set_ring_LED:
            stopAnim = self.ringStopAnim
        else:
            stopAnim = self.stripStopAnim
        wait = 0.005
        while stopAnim == [False]:
            for i in range(255):
                R = color[0] * (i / 255.0)
                G = color[1] * (i / 255.0)
                B = color[2] * (i / 255.0)
                self._set_all_pixels((R, G, B))
                time.sleep(wait)
                self._LED_show()
                if stopAnim == [True]:
                    break
            for i in range(255):
                R = color[0] * ((255 - i) / 255.0)
                G = color[1] * ((255 - i) / 255.0)
                B = color[2] * ((255 - i) / 255.0)
                self._set_all_pixels((R, G, B))
                time.sleep(wait)
                self._LED_show()
                if stopAnim == [True]:
                    break


    def solid(self, color, setStripFunc):
        """
        Changes all LEDs to solid color
        """
        self._set_all_pixels((color[0], color[1], color[2]), setStripFunc)
        self._LED_show()


    def seizure_disco(self, color, setStripFunc):
        """
        Changes all LEDs to a random color
        """
        if setStripFunc == self._set_ring_LED:
            numLEDs = NUM_RING_LEDS
        else:
            numLEDs = NUM_STRIP_LEDS
        for i in range(numLEDs):
            R = random.randint(0,255)
            G = random.randint(0,255)
            B = random.randint(0,255)
            setStripFunc(i, R, G, B)
        self._LED_show()

    def off(self, color, setStripFunc):
        if setStripFunc == self._set_ring_LED:
            numLEDs = NUM_RING_LEDS
        else:
            numLEDs = NUM_STRIP_LEDS
        for i in range(numLEDs):
            setStripFunc(i, 0, 0, 0)
        self._LED_show()

    def boot(self, color, setStripFunc):
        """
        Booting animation
        """
        if setStripFunc == self._set_ring_LED:
            numLEDs = NUM_RING_LEDS
        else:
            numLEDs = NUM_STRIP_LEDS
        for i in range(numLEDs):
            setStripFunc(i, 0, 255, 0)
            self._LED_show()
            time.sleep(1/numLEDs)
        time.sleep(0.3)
        time.sleep(0.3)
        self._set_all_pixels(self.colors["GREEN"])
        time.sleep(1)
        

    def _set_all_pixels(self, color, setStripFunc):
        """
        Sets all pixels to a single
        """
        if setStripFunc == self._set_ring_LED:
            for i in range (NUM_RING_LEDS):
                setStripFunc(i, color[0], color[1], color[2])
        elif setStripFunc == self._set_strip_LED:
            for i in range (NUM_STRIP_LEDS):
                setStripFunc(i, color[0], color[1], color[2])
    
    def _set_ring_LED(self, ledNum, R, G, B):
        if ledNum <= NUM_RING_LEDS - 1 and ledNum >= 0:
            self._set_RGBW_LED(ledNum, R, G, B)
    
    def _set_strip_LED(self, ledNum, R, G, B):
        if ledNum <= NUM_STRIP_LEDS - 1 and ledNum >= 0:
            self._set_RGBW_LED(NUM_RING_LEDS + ledNum, R, G, B)

    def _LED_show(self):
        with self.LEDMutex:
            RGBW_PIXELS.LED_show()

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
        with self.LEDMutex:
            RGBW_PIXELS.set_LED_color(ledNum, newR, newG, newB, newW)
        
        


def main(args=None):
    rclpy.init(args=args)
    led_controller = LEDControllerNode()
    rclpy.spin(led_controller)
    led_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
