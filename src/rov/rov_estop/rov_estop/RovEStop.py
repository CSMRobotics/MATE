import Jetson.GPIO as GPIO
import dbus
import rclpy
from rclpy import Node

class RovEStop(Node):
    def __init__(self):
        super().__init__(node_name="rovestop")

        # setup GPIO names in Board mode
        GPIO.setmode(GPIO.BOARD)
        self.CHANNEL = 12
        GPIO.setup(self.CHANNEL, GPIO.IN)
        
        self.create_timer(0.01, self.main)
    
    def main(self):
        try:
            pinVoltage = GPIO.input(self.CHANNEL)
            if (pinVoltage == GPIO.HIGH):
                #shutdown
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