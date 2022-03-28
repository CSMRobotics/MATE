import Jetson.GPIO as GPIO
import dbus
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

class BaseEStop(Node):
    def __init__(self):
        super().__init__(node_name="baseestop")

        # setup GPIO names in Board mode
        GPIO.setmode(GPIO.BOARD)
        self.CHANNEL = 12
        GPIO.setup(self.CHANNEL, GPIO.OUT)
        GPIO.output(self.CHANNEL, GPIO.LOW)

        self.subscription = self.create_subscription(Bool, "leak", self.leak, 10)
        self.subscription = self.create_subscription(Bool, "estop", self.estop, 10)
    
    def leak(self, msg : Bool) -> None:
        if(msg.data):
            # Cut power
            self.get_logger().fatal("LEAK!!")
            GPIO.output(self.CHANNEL, GPIO.HIGH)
            # Shutdown
            # sys_bus = dbus.SystemBus()
            # ck_srv = sys_bus.get_object('org.freedesktop.login1',
            #                             '/org/freedesktop/login1')
            # ck_iface = dbus.Interface(ck_srv, 'org.freedesktop.login1.Manager')
            # ck_iface.get_dbus_method("PowerOff")(False)
        else:
            # how did we get here?????
            self.get_logger().error("Unknown leak message received")
            pass
    
    def estop(self, msg: Bool) -> None:
        if(msg.data):
            # fatal estop, cut power
            self.get_logger().fatal("ESTOP Received")
            GPIO.output(self.CHANNEL, GPIO.HIGH)
        else:
            # non fatal estop, send stop message to ROV
            self.get_logger().warning("ESTOP Received")
    
def main(args=None):
    rclpy.init()

    estop = BaseEStop()
    rclpy.spin(estop)

    rclpy.shutdown()
    pass

if __name__ == '__main__':
    main()