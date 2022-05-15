import Jetson.GPIO as GPIO
import dbus
import rclpy
from rclpy.node import Node
from time import sleep, time_ns

from std_msgs.msg import Bool

class RovEStop(Node):
    def __init__(self):
        super().__init__(node_name="rovestop")

        self.estop_publisher = self.create_publisher(Bool, "estop", 10)
        self.estop_subscriber = self.create_subscription(Bool, "estop", self.estop, 10)
        self.heartbeat_subscriber = self.create_subscription(Bool, "heartbeat", self.heartbeat_callback, 10)

        self.timeLastHeartbeat = time_ns()
        self.timeSinceLastHeartbeat = 0

        # setup GPIO names in Board mode
        GPIO.setmode(GPIO.BOARD)
        self.CHANNEL = 12
        self.CHANNEL2 = 16
        GPIO.setup(self.CHANNEL, GPIO.IN)
        GPIO.setup(self.CHANNEL2, GPIO.IN)
        
        self.create_timer(0.01, self.main)
    
    def heartbeat_callback(self, msg:Bool):
        time = time_ns()
        self.timeSinceLastHeartbeat  = time - self.timeLastHeartbeat
        self.timeLastHeartbeat = time

        if(self.timeSinceLastHeartbeat >= 200 * 1e6):
            self.get_logger().fatal(f"Heartbeat not detected for {self.timeSinceLastHeartbeat / 1e6} milliseconds")
            self.estop_publisher.publish(Bool(data=False))
    
    def estop(self, msg:Bool):
        #shutdown
        try:
            self.shutdown()
        except Exception as e:
            # log this exception and continue
            self.get_logger().error(e.__traceback__)

            # try again
            self.shutdown()

    def main(self):
        try:
            pinVoltage = GPIO.input(self.CHANNEL)
            pinVoltage2 = GPIO.input(self.CHANNEL2)
            if (pinVoltage == GPIO.HIGH or pinVoltage2 == GPIO.HIGH):
                # notify subscribers that there is a leak
                msg = Bool()
                msg.data = True
                self.estop_publisher.publish(msg)
                # wait to ensure this message is sent over network
                sleep(0.01)
                #shutdown
                self.shutdown()
        except Exception as e:
            # log this exception and continue
            self.get_logger().error(e.__traceback__)
    
    def shutdown(self):
        #TODO: turn off dc-dc converters
        sys_bus = dbus.SystemBus()
        ck_srv = sys_bus.get_object('org.freedesktop.login1',
                                    '/org/freedesktop/login1')
        ck_iface = dbus.Interface(ck_srv, 'org.freedesktop.login1.Manager')
        ck_iface.get_dbus_method("PowerOff")(False)

def main(args=None):
    rclpy.init(args=args)
    
    eStop = RovEStop()
    rclpy.spin(eStop)

    rclpy.shutdown()

if __name__ == "__main__":
    main()