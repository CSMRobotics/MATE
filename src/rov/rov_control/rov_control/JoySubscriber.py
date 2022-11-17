from typing import Dict
from sensor_msgs.msg import Joy
from rclpy.node import Node
from copy import copy


class ParsedJoy:
    # Excludes dpad names
    button_names = [
        "trigger",
        "thumb",
        "button_3",
        "button_4",
        "button_5",
        "button_6",
        "button_7",
        "button_8",
        "button_9",
        "button_10",
        "button_11",
        "button_12",
    ]

    axis_names = [
        "roll",
        "pitch",
        "yaw",
        "throttle",
        "hat_x",
        "hat_y",
    ]

    def __init__(self, msg: Joy, mapping: Dict[str, str], deadzone: float = 0, last_buttons: Dict[str, bool] = None):
        super().__init__()
        self.buttons = {}
        for button_name in self.button_names:
            self.buttons[button_name] = msg.buttons[mapping[button_name]]
        # Parse the button pushes for the dpad
        self.buttons["hat_up"] = msg.axes[mapping["hat_y"]] > 0
        self.buttons["hat_down"] = msg.axes[mapping["hat_y"]] < 0
        self.buttons["hat_left"] = msg.axes[mapping["hat_x"]] < 0
        self.buttons["hat_right"] = msg.axes[mapping["hat_x"]] > 0

        # Toggled buttons default to whatever buttons are currently pushed if last_buttons is empty (e.g., this is our first iteration)
        self.toggled_buttons = copy(self.buttons)

        if last_buttons is not None:
            for button_name in self.buttons.keys():
                self.toggled_buttons[button_name] = self.buttons[button_name] and not last_buttons[button_name]

        self.axes = {}
        for axis_name in self.axis_names:
            self.axes[axis_name] = self.apply_deadzone(msg.axes[mapping[axis_name]], deadzone)

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
        self.joy_subscription = node.create_subscription(Joy, "joy", self.joystick_callback, 2)

    def now_seconds(self):
        return self.node.get_clock().now().nanoseconds / 1000000000.0

    def joystick_callback(self, msg: Joy):
        # self.joystick = ParsedJoy(msg)
        last_buttons = self.latest_joy.buttons if self.latest_joy is not None else None
        self.latest_joy = ParsedJoy(msg, self.mapping, deadzone=self.deadzone, last_buttons=last_buttons)
        self.last_joy_time = self.now_seconds()

    def joy_timeout(self, timeout_seconds):
        return self.last_joy_time == 0 or (self.current_sec - self.last_joy_time) > timeout_seconds

    def delta_time(self):
        min(self.current_sec - self.last_update_time, 0.2)
