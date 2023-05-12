from csm_common_interfaces.msg import JetsonNanoGpio, PinPublisherConfig, PinState
import Jetson.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Don't clobber EStop pins
# PINS = (7, 11, 12, 13, 15, 16, 18, 19, 21, 22, 23, 24, 26, 29, 31, 32, 33, 35, 36, 37, 38, 40)
PINS = (7, 11, 13, 15, 18, 19, 21, 22, 23, 24, 29, 31, 32, 33, 35, 36, 37, 38, 40)

class RovGpio(Node):
    def __init__(self):
        super().__init__(node_name="rovgpio")

        self.getter_publisher = self.create_publisher(JetsonNanoGpio, "rov_gpio", 10)
        self.setter_subscriber = self.create_subscription(PinState, "set_rov_gpio", self.on_set_rov_gpio, 10)
        self.publisher_maker_subscriber = self.create_subscription(PinPublisherConfig, "make_rov_gpio_publisher", self.on_make_rov_gpio_publisher, 10)

        self.create_timer(0.1, self.publish_rov_gpio)

        self.callback_publishers = {}
        self.output_pins = set()

        GPIO.setmode(GPIO.BOARD)

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

class Feeder(Node):
    def __init__(self):
        super().__init__(node_name="feeder")

        self.setter_publisher = self.create_publisher(PinState, "set_rov_gpio", 10)
        self.publisher_maker_publisher = self.create_publisher(PinPublisherConfig, "make_rov_gpio_publisher", 10)

        self.create_timer(0.1, self.feed)

    def feed(self):
        args = list(filter(lambda x: len(x) > 0, input().split(' ')))

        # `set <pin> <high/low/true/false>`
        if args[0] == "set":
            message = PinState()
            message.pin = int(args[1])
            message.state = (args[2] == "true" or args[2] == "high")
            print(f"Setting pin {message.pin} to", "high" if message.state else "low", flush=True)
            self.setter_publisher.publish(message)
        # `publish <pin> <topic_name>`
        elif args[0] == "publish":
            message = PinPublisherConfig()
            message.pin = int(args[1])
            message.name = args[2]
            print(f"Publishing pin {message.pin}'s state to '{message.name}'")
            self.publisher_maker_publisher.publish(message)

class Reader(Node):
    def __init__(self):
        super().__init__(node_name="reader")

        self.getter_subscriber = self.create_subscription(JetsonNanoGpio, "rov_gpio", self.on_get_rov_gpio, 10)

        self.leak_subscriber = self.create_subscription(Bool, "leak", self.on_leak_pin, 10)

        self.known_states = {}
        for pin in PINS:
            self.known_states[pin] = False

    def on_get_rov_gpio(self, message: JetsonNanoGpio):
        for pin in PINS:
            new_state = getattr(message, f"pin{pin}")
            if new_state != self.known_states[pin]:
                print(f"Pin {pin} is now", "high" if new_state else "low", flush=True)
                self.known_states[pin] = new_state

    def on_leak_pin(self, message: Bool):
        if message.data:
            print("LEAK")

def main(args=None):
    rclpy.init(args=args)

    # ex = rclpy.executors.MultiThreadedExecutor()
    # ex.add_node(RovGpio())
    # ex.add_node(Feeder())
    # ex.add_node(Reader())
    # ex.spin()

    rclpy.spin(RovGpio())
    rclpy.shutdown()

if __name__ == "__main__":
    main()