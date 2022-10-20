from typing import Dict
from sensor_msgs.msg import Joy
from rclpy.node import Node


class Buttons:
    def __init__(self):
        self.trigger = False
        self.thumb = False
        self.button_3 = False
        self.button_4 = False
        self.button_5 = False
        self.button_6 = False
        self.button_7 = False
        self.button_8 = False
        self.button_9 = False
        self.button_10 = False
        self.button_11 = False
        self.button_12 = False
        self.hat_up = False
        self.hat_down = False
        self.hat_left = False
        self.hat_right = False


class ParsedJoy(Buttons):
    def __init__(self, msg: Joy, mapping: Dict[str, str], deadzone: float = 0):
        super().__init__()
        self.trigger = bool(msg.buttons[mapping["trigger"]])
        self.thumb = bool(msg.buttons[mapping["thumb"]])
        self.button_3 = bool(msg.buttons[mapping["button_3"]])
        self.button_4 = bool(msg.buttons[mapping["button_4"]])
        self.button_5 = bool(msg.buttons[mapping["button_5"]])
        self.button_6 = bool(msg.buttons[mapping["button_6"]])
        self.button_7 = bool(msg.buttons[mapping["button_7"]])
        self.button_8 = bool(msg.buttons[mapping["button_8"]])
        self.button_9 = bool(msg.buttons[mapping["button_9"]])
        self.button_10 = bool(msg.buttons[mapping["button_10"]])
        self.button_11 = bool(msg.buttons[mapping["button_11"]])
        self.button_12 = bool(msg.buttons[mapping["button_12"]])
        # Parse the button pushes for the dpad
        self.hat_up = msg.axes[mapping["hat_y"]] > 0
        self.hat_down = msg.axes[mapping["hat_y"]] < 0
        self.hat_left = msg.axes[mapping["hat_x"]] < 0
        self.hat_right = msg.axes[mapping["hat_x"]] > 0
        self.roll = self.apply_deadzone(msg.axes[mapping["roll"]], deadzone)
        self.pitch = self.apply_deadzone(msg.axes[mapping["pitch"]], deadzone)
        self.yaw = self.apply_deadzone(msg.axes[mapping["yaw"]], deadzone)
        self.throttle = self.apply_deadzone(msg.axes[mapping["throttle"]], deadzone)
        self.hat_x = self.apply_deadzone(msg.axes[mapping["hat_x"]], deadzone)
        self.hat_y = self.apply_deadzone(msg.axes[mapping["hat_y"]], deadzone)

    def apply_deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0
        else:
            return value


class JoySubscriber:
    # self.get_parameter("deadzone").value
    def __init__(self, node: Node, mapping: Dict, deadzone: float):
        self.mapping = mapping
        self.deadzone = deadzone
        self.node = node
        self.latest_joy = None
        self.last_joy_time = 0
        self.toggled_buttons = Buttons()
        self.joy_subscription = node.create_subscription(Joy, "joy", self.joystick_callback, 2)

    def now_seconds(self):
        return self.node.get_clock().now().nanoseconds / 1000000000.0

    def joystick_callback(self, msg: Joy):
        # self.joystick = ParsedJoy(msg)
        joystick = ParsedJoy(msg, self.mapping, deadzone=self.deadzone)

        for button in dir(self.toggled_buttons):
            # Here, we raise our middle fingers to the Python gods in our
            # religious pursuit of avoiding code duplication, and pray that
            # their retribution is mild
            if button[0] == "_":
                continue
            # Set this button to "Toggled" if
            # A) it's pushed now and
            # B) it wasn't pushed before
            if getattr(joystick, button) and not getattr(self.latest_joy, button):
                setattr(self.toggled_buttons, button, True)

        # self.current_joy = msg
        self.last_joy_time = self.now_seconds()
        self.latest_joy = joystick

    def joy_timeout(self, timeout_seconds):
        return self.last_joy_time == 0 or (self.current_sec - self.last_joy_time) > timeout_seconds

    def delta_time(self):
        min(self.current_sec - self.last_update_time, 0.2)
