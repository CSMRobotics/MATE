import Jetson.GPIO as GPIO
import dbus
import rclpy
from rclpy.node import Node
from time import sleep

from std_msgs.msg import Bool

class BaseEStop(Node):
    def __init__(self):
        super().__init__(node_name="baseestop")

        # setup GPIO names in Board mode
        GPIO.setmode(GPIO.BOARD)
        self.CHANNEL = 12
        GPIO.setup(self.CHANNEL, GPIO.OUT)
        GPIO.output(self.CHANNEL, GPIO.LOW)

        self.subscription = self.create_subscription(Bool, "estop", self.estop, 10)
    
    def leak(self, msg : Bool) -> None:
        # Cut power
        self.get_logger().fatal("LEAK!!")
        self.get_logger().fatal("UNSAFE SHUTDOWN MAY OCCUR, VERIFY INTEGRITY OF FILESYSTEM")
        GPIO.output(self.CHANNEL, GPIO.HIGH)
        # ensure shutdown pin is pulled high
        sleep(0.1)
        # Shutdown (might be enough time with caps to safely shutdown)
        sys_bus = dbus.SystemBus()
        ck_srv = sys_bus.get_object('org.freedesktop.login1',
                                    '/org/freedesktop/login1')
        ck_iface = dbus.Interface(ck_srv, 'org.freedesktop.login1.Manager')
        ck_iface.get_dbus_method("PowerOff")(False)

    
    def estop(self, msg: Bool) -> None:
        self.get_logger().warning("ESTOP Received by BaseStation")
        if(msg.data):
            # fatal estop, cut power
            self.leak()
        else:
            # non fatal estop
            pass
    
def main(args=None):
    rclpy.init()

    estop = BaseEStop()
    rclpy.spin(estop)

    rclpy.shutdown()

if __name__ == '__main__':
    main()