import Jetson.GPIO as GPIO
import dbus
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

class RovEStop(Node):
    def __init__(self):
        super().__init__(node_name="rovestop")

        self.leak_publisher = self.create_publisher(Bool, "leak", 10)
        self.estop_publisher = self.create_publisher(Bool, "estop", 10)

        # setup GPIO names in Board mode
        GPIO.setmode(GPIO.BOARD)
        self.CHANNEL = 12
        GPIO.setup(self.CHANNEL, GPIO.IN)
        
        self.create_timer(0.01, self.main)
    
    def main(self):
        try:
            pinVoltage = GPIO.input(self.CHANNEL)
            if (pinVoltage == GPIO.HIGH):
                # notify subscribers that there is a leak
                msg = Bool()
                msg.data = True
                self.leak_publisher.publish(msg)
                self.estop_publisher.publish(msg)
                # TODO: Do we wait to ensure this message is sent over network??
                # sleep(0.1)
                #shutdown
                # TODO: does this work inside docker container
                sys_bus = dbus.SystemBus()
                ck_srv = sys_bus.get_object('org.freedesktop.login1',
                                            '/org/freedesktop/login1')
                ck_iface = dbus.Interface(ck_srv, 'org.freedesktop.login1.Manager')
                ck_iface.get_dbus_method("PowerOff")(False)
        except:
            # maybe log this
            pass

def main(args=None):
    rclpy.init(args=args)
    
    eStop = RovEStop()
    rclpy.spin(eStop)

    rclpy.shutdown()

if __name__ == "__main__":
    main()