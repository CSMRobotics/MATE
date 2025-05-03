from time import sleep
from . import ms5837

import rclpy
from rclpy.node import Node

from rov_interfaces.msg import Bar30Data

class Bar30_Node(Node):
    OSR = ms5837.OSR_1024

    def __init__(self):
        super().__init__("bar30")
        self._publisher = self.create_publisher(Bar30Data, "bar30", 10)
        self.sensor = ms5837.MS5837_30BA(1)
        self.sensor.setFluidDensity(992.72) # rough estimate of density of pool water (we arent going to use sat liquid tables)
        self.sensor.init()
        sleep(0.01)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        self.sensor.read(self.OSR)
        msg = Bar30Data()
        msg.altitude = self.sensor.altitude()
        msg.depth = self.sensor.depth()
        msg.pressure = self.sensor.pressure() # mbar
        msg.temperature_c = self.sensor.temperature() # deg C
        self._publisher.publish(msg)

def main():
    rclpy.init()

    bar30 = Bar30_Node()
    rclpy.spin(bar30)

    rclpy.shutdown()


if __name__ == '__main__':
    main()