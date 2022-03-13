from time import sleep
import ms5837

import rclpy
from rclpy import Node

from rov_interfaces.msg import bar02_data

class Bar02_Node(Node):
    OSR = ms5837.OSR_1024

    def __init__(self):
        super().__init__("bar02")
        self._publisher = self.create_publisher(bar02_data, "bar02", 10)
        self.sensor = ms5837.MS5837_02BA()
        self.sensor.setFluidDensity(992.72) # rough estimate of density of pool water
        self.sensor.init()
        sleep(0.01)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        self.sensor.read(self.OSR)
        msg = bar02_data()
        msg.altitude = self.sensor.altitude()
        msg.depth = self.sensor.depth()
        msg.pressure = self.sensor.pressure() # mbar
        msg.temperature_c = self.sensor.temperature() # deg C
        self._publisher.publish(msg)

def main():
    rclpy.init()

    bar02 = Bar02_Node()
    rclpy.spin(bar02)

    rclpy.shutdown()


if __name__ == '__main__':
    main()