from time import sleep
from . import tsys01

import rclpy
from rclpy.node import Node

from rov_interfaces.msg import TSYS01Data

class Tsys01_Node(Node):
    def __init__(self):
        super().__init__("tsys01")
        self._publisher = self.create_publisher(TSYS01Data, "tsys01", 10)
        self.sensor = tsys01.TSYS01()
        self.sensor.init()
        sleep(0.01)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        self.sensor.read()
        msg = TSYS01Data()
        msg.temperature_c = self.sensor.temperature() # deg C
        self._publisher.publish(msg)

def main():
    rclpy.init()

    temp_node = Tsys01_Node()
    rclpy.spin(temp_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()