import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from rov_control.JoySubscriber import JoySubscriber
from rov_control.JoySubscriber import ParsedJoy
from rov_interfaces.msg import ManipulatorSetpoints, ThrusterSetpoints
from transitions import Machine
from std_msgs.msg import String

from enum import Enum


class States(Enum):
    shutdown = 0
    paused = 1
    teleop_drive = 2
    teleop_manip = 3

def declare_parameters(node : Node):
    node.declare_parameters("mapping",
        [
            ("trigger", 0),
            ("trigger", 0),
            ("thumb", 1),
            ("button_3", 2),
            ("button_4", 3),
            ("button_5", 4),
            ("button_6", 5),
            ("button_7", 6),
            ("button_8", 7),
            ("button_9", 8),
            ("button_10", 9),
            ("button_11", 10),
            ("button_12", 11),
            ("roll", 0),
            ("pitch", 1),
            ("yaw", 2),
            ("throttle", 3),
            ("hat_x", 4),
            ("hat_y", 5),
        ]
    )

    node.declare_parameters("",
        [
            ("chicken_speed", 10),
            ("wrist_speed", 10),
            ("level_min", -90),
            ("level_initial", 0),
            ("level_max", 90),
            ("elbow_min", -90),
            ("elbow_initial", 0),
            ("elbow_max", 90),
            ("wrist_min", -90),
            ("wrist_initial", 0),
            ("wrist_max", 90),
            ("clamp_initial", 0),
            ("deadzone", 0),
            ("joy_update_hz", 10),
            ("joy_timeout", 0.5)
        ]
    )


class ROV_Control(Node):
    def __init__(self):
        super().__init__("rov_control")
        declare_parameters(self)
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
        self.joy_subscriber = JoySubscriber(self, self.mapping, 0.02)
        self.create_timer(1 / self.get_parameter("joy_update_hz").value, self.joy_update)
        self.manipulator_setpoint_pub = self.create_publisher(ManipulatorSetpoints, "manipulator_setpoints", 2)
        self.thruster_setpoint_pub = self.create_publisher(ThrusterSetpoints, "thruster_setpoints", 2)
        self.state_pub = self.create_publisher(String, "rov_control_state", 2)
        self.create_timer(1, lambda: self.state_pub.publish(String(data=self.state.name)))
        self.create_subscription(Bool, "estop", self.estop_callback, 0)
        self.state = States.teleop_drive

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

    def joy_update(self):
        if self.joy_subscriber.joy_timeout(self.get_parameter("joy_timeout").value):
            return
        delta_time = self.joy_subscriber.delta_time()
        joystick = self.joy_subscriber.latest_joy
        self.toggled_buttons = joystick.toggled_buttons

        # if mode switch button has been pressed, toggle the mode if you are in teleop
        if self.state in [States.teleop_manip, States.teleop_drive] and self.toggled_buttons["button_8"]:
            self.teleop_mode_switch()

        # update teleop modes
        if self.state == States.teleop_manip:
            self.do_manip_setpoint_update(joystick, delta_time)
            self.thruster_setpoint_pub.publish(ThrusterSetpoints())
        elif self.state == States.teleop_drive:
            self.do_thrust_setpoint_update(joystick)
            self.manipulator_setpoint_pub.publish(self.last_manip)

    def param_clamp(self, value, name):
        return min(max(value, float(self.get_parameter(name + "_min").value)), float(self.get_parameter(name + "_max").value))

    def teleop_mode_switch(self):
        return

    def do_manip_setpoint_update(self, joystick: ParsedJoy, delta_time: float):
        # NOTE: As of 10/19/22, we're looking into redoing the manipulator, so this is probably useless
        # ALSO TODO: Since we changed the control organization a little so that most logic related to thrust is done in the flight_controller,
        # ... we should also move all logic related to manipulation into the manipulator node, and only send raw joystick values (meaning [-1, 1])
        # Also TODO: When doing that last thing don't forget to change estop handling to match. (look below)
        manip_setpoints = self.last_manip
        chicken_speed = self.get_parameter("chicken_speed").value
        manip_setpoints["elbow"] += joystick["pitch"] * chicken_speed * delta_time
        manip_setpoints["elbow"] = self.param_clamp(manip_setpoints["elbow"], "elbow")
        # chicken
        if joystick["thumb"]:
            # Do IK here :)
            manip_setpoints["level"] = 180 - manip_setpoints.elbow
        else:
            manip_setpoints["level"] += joystick["hat_y"] * chicken_speed * delta_time
        manip_setpoints["level"] = self.param_clamp(manip_setpoints["level"], "level")

        if self.toggled_buttons["button_3"]:
            manip_setpoints.clamp = 1.0 - manip_setpoints.clamp

        manip_setpoints["wrist"] += joystick["roll"] * self.get_parameter("wrist_speed").value * delta_time
        manip_setpoints["wrist"] = self.param_clamp(manip_setpoints["wrist"], "wrist")

        self.manipulator_setpoint_pub.publish(manip_setpoints)
        self.last_manip = manip_setpoints

    def do_thrust_setpoint_update(self, joystick: ParsedJoy):
        # All values default to zero
        thrust_setpoints = ThrusterSetpoints()

        if joystick["thumb"]:
            # Rotation mode
            thrust_setpoints.omegax = -float(joystick["pitch"])
            thrust_setpoints.omegay = -float(joystick["roll"])
            thrust_setpoints.omegaz = -float(joystick["yaw"])
        else:
            # Translation mode
            thrust_setpoints.vx = float(joystick["pitch"])
            thrust_setpoints.vy = -float(joystick["roll"])
            thrust_setpoints.vz = float(joystick["throttle"])

        self.thruster_setpoint_pub.publish(thrust_setpoints)

    def estop_fatal(self):
        return
    
    def estop_normal(self):
        return

    def estop_callback(self, msg):
        if msg.data:
            self.estop_fatal()
        else:
            self.estop_normal()

    def stop_all_motors(self):
        self.thruster_setpoint_pub.publish(ThrusterSetpoints())  # All values will default to zero
        # Manipulator values are position-based, so we just stop writing new values to cause it to stop
        # "Not writing new values" is accomplished by changing state, which has already happened now that we've entered 'paused'

    def on_enter_paused(self):
        # Stop motors and all that
        self.stop_all_motors()
        self.get_logger().warn("ROV_Control has entered the paused state")
        pass

    def on_enter_shutdown(self):
        # The sky is falling and we're all going to die
        self.stop_all_motors()
        self.get_logger().fatal("ROV_Control has entered emergency shutdown")
        pass


def main():
    rclpy.init(args=None)
    rclpy.spin(ROV_Control())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
