# lifted from this lovely repo: https://github.com/seitomatsubara/Jetson-nano-WS2812-LED-

import spidev
import sys
 
class SPItoWS():
    def __init__(self, ledc, bus=1, device=0):
        self.led_count = ledc
        self.X = '' # X is signal of WS281x
        for i in range(self.led_count):
            self.X = self.X + "100100100100100100100100100100100100100100100100100100100100100100100100"
        self.led_brightness = 1.0
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 2400000

    def __del__(self):
        self.spi.close()
        print("destructor")
        
    def _Bytesto3Bytes(self, num, RGB): # num is number of signal, RGB is 8 bits (1 byte) str
        for i in range(8):
            if RGB[i] == '0':
                self.X = self.X[:num * 3 * 8 + i * 3] + '100' + self.X[num * 3 * 8 + i * 3 + 3:]
            elif RGB[i] == '1':
                self.X = self.X[:num * 3 * 8 + i * 3] + '110' + self.X[num * 3 * 8 + i * 3 + 3:]
    
    def _BytesToHex(self, Bytes):
        return ''.join(["0x%02X " % x for x in Bytes]).strip()
    
    def LED_show(self):
            Y = []
            for i in range(self.led_count * 9):
                Y.append(int(self.X[i*8:(i+1)*8],2))
            WS = self._BytesToHex(Y)
            self.spi.xfer3(Y, 2400000,0,8)

    def set_LED_color(self, led_num, R, G, B):
        if (R > 255 or G > 255 or B > 255):
            print("Invalid Value: RGB is over 255\n")
            sys.exit(1)
        if (led_num > self.led_count - 1):
            print("Invalid Value: The number is over the number of LED")
            sys.exit(1)
        RR = format(round(R * self.led_brightness), '08b')
        GG = format(round(G * self.led_brightness), '08b')
        BB = format(round(B * self.led_brightness), '08b')
        self._Bytesto3Bytes(led_num * 3, GG)
        self._Bytesto3Bytes(led_num * 3 + 1, RR)
        self._Bytesto3Bytes(led_num * 3 + 2, BB)

        # debug
        # print("led_num: %d, R: %d G: %d B: %d" % (led_num, round(R * self.led_brightness), round(G * self.led_brightness), round(B * self.led_brightness)))

    def LED_OFF_ALL(self):
        self.X = ''
        for i in range(self.led_count):
            self.X = self.X + "100100100100100100100100100100100100100100100100100100100100100100100100"
        self.LED_show()

    def set_brightness(self, brightness):
        self.led_brightness = brightness


if __name__ == "__main__":
    import time
    LED_COUNT = 3
    sig = SPItoWS(LED_COUNT)
    sig.RGBto3Bytes(0, 255, 0, 0)
    sig.RGBto3Bytes(1, 0, 255, 0)
    sig.RGBto3Bytes(2, 0, 0, 255)
    sig.LED_show()
    time.sleep(1)
    sig.LED_OFF_ALL()