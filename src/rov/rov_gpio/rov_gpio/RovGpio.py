from csm_common_interfaces.msg import JetsonNanoGpio, PinPublisherConfig, PinState
import Jetson.GPIO as GPIO
import rclpy
from std_msgs.msg import Bool

# Don't clobber EStop pins
# PINS = (7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40)
PINS = (7, 11, 13, 15, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40)

class RovGpio(rclpy.Node):
    def __init__(self):
        super().__init__(node_name="rovgpio")

        self.getter_publisher = self.create_publisher(JetsonNanoGpio, "rov_gpio", 10)
        self.setter_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)
        self.publisher_maker_subscriber = self.create_subscription(PinPublisherConfig, "make_rov_gpio_publisher", self.on_make_rov_gpio_publisher, 10)

        self.create_timer(0.1, self.publish_rov_gpio)

        self.callback_publishers = {}
        self.output_pins = set()

        for pin in PINS:
            GPIO.setup(pin, GPIO.IN)

    def on_make_rov_gpio_publisher(self, message: PinPublisherConfig):
        if message.pin not in PINS:
            pass

        publisher = self.create_publisher(Bool, message.name, 10)

        callbacks = self.callback_publishers.get(message.pin)
        if callbacks == None:
            self.callback_publishers[message.pin] = [publisher]
        else:
            self.callback_publishers[message.pin].append(publisher)

    def on_set_rov_gpio(self, message: PinState):
        if message.pin not in PINS:
            pass

        if message.pin not in self.output_pins:
            GPIO.setup(message.pin, GPIO.OUT)
            self.output_pins.add(message.pin)

        GPIO.output(message.pin, GPIO.HIGH if message.state else GPIO.LOW)

    def publish_rov_gpio(self):
        message = JetsonNanoGpio()

        for pin in PINS:
            state = True if GPIO.input(pin) == GPIO.HIGH else False
            state_message = Bool()
            state_message.data = state

            callbacks = self.callback_publishers.get(pin)
            if callbacks != None:
                for callback in callbacks:
                    callback.publish(state_message)

            setattr(message, f"pin{pin}", state)
        
        self.getter_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RovGpio())
    rclpy.shutdown()

if __name__ == "__main__":
    main()