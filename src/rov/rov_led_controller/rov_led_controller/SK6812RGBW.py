import spidev
import sys
import time

class SPItoSK():
    def __init__(self, numLeds, bus = 1, device = 0):
        self.ledCount = numLeds
        self.ledBrightness = 1.0
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 2400000
        # string of bits to send to spi
        self.binMsg = 0b100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100
        for i in range(numLeds - 1):
            # 96 bits per led, 3 bits per real bit
            self.binMsg = (self.binMsg << 96) | 0b100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100

    def __del__(self):
        self.spi.close()
        print("destructor")
    
    def set_LED_color(self, ledNum, R, G, B, W):
        """
        Adjust self.binMsg to the color desired
        :param ledNum: the led to change starting with index 0
        :param R, G, B, W: RGBW values from 0-255
        """
        if (R > 255) or (G > 255) or (B > 255) or (W > 255) or (R < 0) or (G < 0) or (B < 0) or (W < 0):
            print("RGBW values must be in the range 0-255")
            sys.exit(-1)
        msgStartPos = (ledNum * 96) + 2
        self._formatBinMsg(msgStartPos, int(round(R * self.ledBrightness)))
        self._formatBinMsg(msgStartPos + 24, int(round(G * self.ledBrightness)))
        self._formatBinMsg(msgStartPos + (24 * 2), int(round(B * self.ledBrightness)))
        self._formatBinMsg(msgStartPos + (24 * 3), int(round(W * self.ledBrightness)))

        # debug
        print("led_num: %d, R: %d G: %d B: %d W: %d" % (ledNum, round(R * self.ledBrightness), round(G * self.ledBrightness), round(B * self.ledBrightness), round(W * self.ledBrightness)))

    def LED_show(self):
        """
        Signals the LEDs
        """
        bytesToSend = []
        while self.binMsg:
            bytesToSend.insert(0, self.binMsg & ((1 << 8) - 1))  # Extract lowest 8 bits
            self.binMsg >>= 8  # Shift right by 8 bits
        self.spi.xfer(bytesToSend, delay_usec = 0, bits_per_word = 8)
        time.sleep(80e-6)

    def set_brightness(self, brightness):
        """
        Set the global brightness of LEDs
        :param brightness: Brightness of LEDs 0-1, clamps if values are outside the range
        """
        brightness = max(0.0, min(brightness, 1.0))
        self.ledBrightness = brightness

    def LED_OFF_ALL(self):
        """
        Turns off LEDs
        """
        self.binMsg = 0b100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100
        for i in range(self.ledCount - 1):
            # 96 bits per led, 3 bits per real bit
            self.binMsg = (self.binMsg << 96) | 0b100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100100
        self.LED_show()
                
    def _formatBinMsg(self, startPos, colorNum):
        number = format(colorNum, '08b')
        for i in range(8):
            msgIndexPos = startPos + (3 * i)
            colorBit = number[i]
            if (colorBit == "0") and bin(self.binMsg)[msgIndexPos:msgIndexPos + 3] == "110":
                self._flipBit(msgIndexPos + 1)
            elif (colorBit == "1") and bin(self.binMsg)[msgIndexPos:msgIndexPos + 3] == "100":
                self._flipBit(msgIndexPos)

    def _flipBit(self, position):
        mask = 1 << len(bin(self.binMsg)[2:]) - position
        self.binMsg = self.binMsg ^ mask

    
if __name__ == "__main__":
    ledStrip = SPItoSK(1)
    ledStrip.set_LED_color(0, 255, 255, 255, 255)
    ledStrip.LED_show()