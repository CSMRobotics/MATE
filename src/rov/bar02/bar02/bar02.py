from time import sleep
from . import ms5837

import rclpy
from rclpy.node import Node

from rov_interfaces.msg import Bar02Data

class Bar02_Node(Node):
    def __init__(self):
        super().__init__("bar02")
        self._publisher = self.create_publisher(Bar02Data, "bar02", 10)
        self.sensor = ms5837.MS5837_02BA()
        self.sensor.setFluidDensity(992.72) # rough estimate of density of pool water (we arent going to use sat liquid tables)
        self.sensor.init()
        sleep(0.01)
        # data sheet says 17.2 ms max time for OSR 8192 (roughly 60 hz)
        self.timer = self.create_timer(18/1000, self.timer_callback)

    def timer_callback(self):
        self.sensor.read()
        msg = Bar02Data()
        msg.altitude = self.sensor.altitude()
        msg.depth = self.sensor.depth()
        msg.pressure = self.sensor.pressure(conversion=ms5837.UNITS_kPa) # kPa
        msg.temperature_c = self.sensor.temperature() # deg C
        self._publisher.publish(msg)

def main():
    rclpy.init()

    rclpy.spin(Bar02_Node())

    rclpy.shutdown()


if __name__ == '__main__':
    main()