from fnmatch import translate
from typing import Dict
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from rov_interfaces.msg import ManipulatorSetpoints, ThrusterSetpoints
from transitions import Machine
from std_msgs.msg import String

from enum import Enum


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
        self.roll = self.deadzone(msg.axes[mapping["roll"]], deadzone)
        self.pitch = self.deadzone(msg.axes[mapping["pitch"]], deadzone)
        self.yaw = self.deadzone(msg.axes[mapping["yaw"]], deadzone)
        self.throttle = self.deadzone(msg.axes[mapping["throttle"]], deadzone)
        self.hat_x = self.deadzone(msg.axes[mapping["hat_x"]], deadzone)
        self.hat_y = self.deadzone(msg.axes[mapping["hat_y"]], deadzone)

    def deadzone(self, value, deadzone):
        if abs(value) < deadzone:
            return 0
        else:
            return value


class States(Enum):
    shutdown = 0
    paused = 1
    teleop_drive = 2
    teleop_manip = 3


class ROV_Control(Node):
    def __init__(self):
        super().__init__("rov_control", automatically_declare_parameters_from_overrides=True)
        self.thruster_setpoint_publisher = self.create_publisher(ThrusterSetpoints, "thruster_setpoints", 10)
        self.machine = Machine(self, states=States, initial=States.teleop_drive)
        self.machine.add_transition("estop_normal", "*", States.paused)
        self.machine.add_transition("estop_fatal", "*", States.shutdown)
        self.machine.add_transition("teleop_mode_switch", States.teleop_manip, States.teleop_drive)
        self.machine.add_transition("teleop_mode_switch", States.teleop_drive, States.teleop_manip)
        self.mapping = self.dictionaryafy(self.get_parameters_by_prefix("mapping"))
        self.last_manip = ManipulatorSetpoints(
            clamp=float(self.get_parameter("clamp_initial").value),
            level=float(self.get_parameter("level_initial").value),
            wrist=float(self.get_parameter("wrist_initial").value),
            elbow=float(self.get_parameter("elbow_initial").value),
        )
        self.last_joy_time = 0
        self.last_update_time = 0
        self.toggled_buttons = Buttons()
        self.last_joy = None
        self.joy_subscription = self.create_subscription(Joy, "joy", self.joystick_callback, 2)
        self.create_timer(1 / self.get_parameter("joy_update_hz").value, self.joy_update)
        self.manipulator_setpoint_pub = self.create_publisher(ManipulatorSetpoints, "manipulator_setpoints", 2)
        self.thruster_setpoint_pub = self.create_publisher(ThrusterSetpoints, "thruster_setpoints", 2)
        self.state_pub = self.create_publisher(String, "rov_control_state", 2)
        self.create_timer(1, lambda: self.state_pub.publish(String(data=self.state.name)))
        self.create_subscription(Bool, "estop", self.estop_callback, 0)

    def dictionaryafy(self, in_dict):
        tree = {}
        for key, value in in_dict.items():
            t = tree
            parts = key.split(".")
            for part in parts[:-1]:
                t = t.setdefault(part, {})
            t[parts[-1]] = value.value
        return tree

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1000000000.0

    def joystick_callback(self, msg: Joy):
        # self.joystick = ParsedJoy(msg)
        joystick = ParsedJoy(msg, self.mapping, deadzone=self.get_parameter("deadzone").value)

        for button in dir(self.toggled_buttons):
            # Here, we raise our middle fingers to the Python gods in our
            # religious pursuit of avoiding code duplication, and pray that
            # their retribution is mild
            if button[0] == "_":
                continue
            # Set this button to "Toggled" if
            # A) it's pushed now and
            # B) it wasn't pushed before
            if getattr(joystick, button) and not getattr(self.last_joy, button):
                setattr(self.toggled_buttons, button, True)

        # self.current_joy = msg
        self.last_joy_time = self.now_seconds()
        self.last_joy = joystick

    def joy_update(self):
        if not self.last_joy:
            return

        current_sec = self.now_seconds()
        if self.last_joy_time == 0 or (current_sec - self.last_joy_time) > self.get_parameter("joy_timeout").value:
            return
        delta_time = min(current_sec - self.last_update_time, 0.2)

        # joystick = ParsedJoy(self.current_joy, self.mapping, self.last_joy, )
        joystick = self.last_joy

        # if mode switch button has been pressed, toggle the mode if you are in teleop
        if self.state in [States.teleop_manip, States.teleop_drive] and self.toggled_buttons.button_8:
            self.teleop_mode_switch()

        # update teleop modes
        if self.state == States.teleop_manip:
            self.do_manip_setpoint_update(joystick, delta_time)
            self.thruster_setpoint_pub.publish(ThrusterSetpoints())
        elif self.state == States.teleop_drive:
            self.do_thrust_setpoint_update(joystick)
            self.manipulator_setpoint_pub.publish(self.last_manip)

        self.last_joy_time = current_sec
        self.last_joy = joystick
        self.toggled_buttons = Buttons()

    def param_clamp(self, value, name):
        return min(max(value, float(self.get_parameter(name + "_min").value)), float(self.get_parameter(name + "_max").value))

    def do_manip_setpoint_update(self, joystick: ParsedJoy, delta_time: float):
        manip_setpoints = self.last_manip
        chicken_speed = self.get_parameter("chicken_speed").value
        manip_setpoints.elbow += joystick.pitch * chicken_speed * delta_time
        manip_setpoints.elbow = self.param_clamp(manip_setpoints.elbow, "elbow")
        # chicken
        if joystick.thumb:
            # Do IK here :)
            manip_setpoints.level = 180 - manip_setpoints.elbow
        else:
            manip_setpoints.level += joystick.hat_y * chicken_speed * delta_time
        manip_setpoints.level = self.param_clamp(manip_setpoints.level, "level")

        if self.toggled_buttons.button_3:
            manip_setpoints.clamp = 1.0 - manip_setpoints.clamp

        manip_setpoints.wrist += joystick.roll * self.get_parameter("wrist_speed").value * delta_time
        manip_setpoints.wrist = self.param_clamp(manip_setpoints.wrist, "wrist")

        self.manipulator_setpoint_pub.publish(manip_setpoints)
        self.last_manip = manip_setpoints

    def do_thrust_setpoint_update(self, joystick: ParsedJoy):
        thrust_setpoints = ThrusterSetpoints()
        translate_speed = self.get_parameter("translate_speed").value
        rotate_speed = self.get_parameter("rotate_speed").value

        if joystick.thumb:
            # Rotation mode
            thrust_setpoints.omegax = -joystick.pitch * rotate_speed
            thrust_setpoints.omegay = -joystick.roll * rotate_speed
            thrust_setpoints.omegaz = -joystick.yaw * rotate_speed
        else:
            # Translation mode
            thrust_setpoints.vx = -joystick.roll * translate_speed
            thrust_setpoints.vy = joystick.pitch * translate_speed
            thrust_setpoints.vz = joystick.throttle * translate_speed

        self.thruster_setpoint_pub.publish(thrust_setpoints)

    def estop_callback(self, msg):
        if msg.data:
            self.estop_fatal()
        else:
            self.estop_normal()

    def on_enter_paused(self):
        # Stop motors and all that
        self.get_logger().warn("ROV_Control has entered the paused state")
        pass

    def on_enter_shutdown(self):
        # The sky is falling and we're all going to die
        self.get_logger().fatal("ROV_Control has entered emergency shutdown")
        pass


def main():
    rclpy.init(args=None)
    rclpy.spin(ROV_Control())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
