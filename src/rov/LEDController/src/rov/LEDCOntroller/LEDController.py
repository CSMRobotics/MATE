from common.csm_common_interfaces.msg import PinState
import rclpy
from rclpy.node import Node
import neopixel

class RovGpio(Node):
    def __init__(self):
        super().__init__(node_name="LEDController")
        self.setter_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)


if __name__ == '__main__':
    main()
