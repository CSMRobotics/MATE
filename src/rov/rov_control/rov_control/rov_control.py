from csm_common_interfaces.msg import PinState
from enum import Enum
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rov_interfaces.msg import ManipulatorSetpoints, ThrusterSetpoints
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from transitions import Machine
from typing import Any, Dict, List, Optional, Tuple, TypeVar, Union

K = TypeVar('K')
V = TypeVar('V')

RecursiveDict = Dict[K, Union[V, "RecursiveDict"]]

ParameterDict = RecursiveDict[str, Optional[Union[
    Parameter.Type,
    Tuple[Any, ParameterDescriptor]
]]]

def namespace_flatten_dict(recursive_dict: RecursiveDict[str, V]) -> Dict[str, V]:
    output_dict: Dict[str, V] = {}

    for key, value in recursive_dict.items():
        if isinstance(value, dict):
            output_dict.update({f"{key}.{k}": v for k, v in namespace_flatten_dict(value).items()})
        else:
            output_dict[key] = value

    return output_dict

def declare_parameter_dict(node: Node, parameters: ParameterDict) -> List[Parameter]:
    return node.declare_parameters("", list(namespace_flatten_dict(parameters).items()))

class Timeout:
    def __init__(self, node: Node, seconds: float = 0.0, nanoseconds: int = 0):
        self._node = node
        self._end_time = node.get_clock().now().nanoseconds + int(seconds * 1.0e9) + nanoseconds
    
    def passed(self) -> bool:
        return self._node.get_clock().now().nanoseconds > self._end_time

class Button:
    # `state` is either the current state, or the number of historical state transitions, starting from `False``
    def __init__(self, name: Optional[str], state: Union[bool, int]):
        self._name = name
        self._transitions = int(state)

    @property
    def name(self) -> Optional[str]:
        return self._name
    
    @property
    def state(self) -> bool:
        return bool(self._transitions % 2)

    @property
    def toggled(self) -> bool:
        return bool(self % 2)
    
    # The boolean form is the current state
    def __bool__(self) -> bool:
        return self.state
    
    # `Button % rhs` is the total number of presses mod `rhs`, see `toggled()`
    def __mod__(self, rhs: int) -> int:
        return ((self._transitions + 1) // 2) % rhs

    def __eq__(self, rhs: bool) -> bool:
        return self.state.__eq__(rhs)

    def __ne__(self, rhs: bool) -> bool:
        return self.state.__ne__(rhs)

class Axis:
    def __init__(self, name: Optional[str], value: float):
        self._name = name
        self._value = value

    def deadzoned(self, deadzone: float) -> "Axis":
        return Axis(self.name, 0 if abs(self.value) <= deadzone else self.value)
    
    @property
    def name(self) -> Optional[str]:
        return self._name
    
    @property
    def value(self) -> float:
        return self._value

    # If `self` is a hat axis where up is positive, then `+self` would be the up hat button
    def __pos__(self) -> Button:
        return Button(self.name, self > 0.0)

    # If `self` is a hat axis where up is positive, then `-self` would be the down hat button
    def __neg__(self) -> Button:
        return Button(self.name, self < 0.0)

    def __invert__(self) -> "Axis":
        return Axis(self.name, -self.value)

    def __lt__(self, rhs: Union[float, int]) -> bool:
        return self.value.__lt__(rhs)

    def __le__(self, rhs: Union[float, int]) -> bool:
        return self.value.__le__(rhs)

    def __eq__(self, rhs: Union[float, int]) -> bool:
        return self.value.__eq__(rhs)

    def __ne__(self, rhs: Union[float, int]) -> bool:
        return self.value.__ne__(rhs)

    def __gt__(self, rhs: Union[float, int]) -> bool:
        return self.value.__gt__(rhs)

    def __ge__(self, rhs: Union[float, int]) -> bool:
        return self.value.__ge__(rhs)

class Joystick:
    def __init__(
        self,
        node: Node,
        joystick_topic_name_l: str,
        joystick_topic_name_r: str,
        axis_names: Union[List[str], Dict[str, int]],
        button_names: Union[List[str], Dict[str, int]]
    ):
        # TODO: Un-emergency fix this to actually use two joystick instances, each with their own topic name, rather than
        # all this functionality shoved into the same node bc I didn't feel like editting the button axis-offset mapping stuff.
        self._axis_aliases: Dict[str, int] = {}
        if isinstance(axis_names, list):
            self._axis_aliases = {name: index for index, name in enumerate(axis_names)}
        elif isinstance(axis_names, dict):
            self._axis_aliases = {name: index for name, index in axis_names.items() if index >= 0}

        self._button_aliases: Dict[str, int] = {}
        if isinstance(button_names, list):
            self._button_aliases = {name: index for index, name in enumerate(button_names)}
        elif isinstance(button_names, dict):
            self._button_aliases = {name: index for name, index in button_names.items() if index >= 0}


        # # number of axes and buttons for each joystick, used for recognizing different joystick messages
        self._num_axes: List[int] = [6, 10]
        self._num_buttons: List[int] = [12, 79]

        # # values of joystick
        self._axis_values: List[float] = [0] * sum(self._num_axes)
        self._button_states: List[bool] = [0] * sum(self._num_buttons)
        self._button_transitions: List[int] = [0] * sum(self._num_buttons)

        self._joystick_update_subscription = node.create_subscription(
            Joy,
            joystick_topic_name_r,
            lambda msg: self.on_joystick_message(msg, 0, 0),
            10
        )
        # TODO Seriously un-hardcode these offsets
        # Offset left controller by number of buttons+axes on right control bc that's how the  mapping is rn
        self._joystick_update_subscription = node.create_subscription(
            Joy,
            joystick_topic_name_l,
            lambda msg: self.on_joystick_message(msg, self._num_buttons[0], self._num_axes[0]),
            10
        )
        
    def axis(self, name_or_index: Union[str, int]) -> Axis:
        if type(name_or_index) == str and self._axis_aliases.get(name_or_index) != None:
            try:
                return Axis(name_or_index, self._axis_values[self._axis_aliases[name_or_index]])
            except:
                return Axis(None, 0.0)
        elif type(name_or_index) == int and 0 <= name_or_index < len(self._axis_values):
            return Axis(None, self._axis_values[name_or_index])
        else:
            return Axis(None, 0.0)

    def button(self, name_or_index: Union[str, int]) -> Button:
        if type(name_or_index) == str and self._button_aliases.get(name_or_index) != None:
            try:
                return Button(name_or_index, self._button_transitions[self._button_aliases[name_or_index]])
            except:
                return Button(None, False)
        elif type(name_or_index) == int and 0 <= name_or_index < len(self._button_transitions):
            return Button(None, self._button_transitions[name_or_index])
        else:
            return Button(None, False)

    def on_joystick_message(self, message: Joy, button_offset:int, axes_offset:int):
        # Check length of message to determine which joystick
        # if len(message.axes) not in self._num_axes:
        #     self._num_axes.append(len(message.axes))
        #     self._axis_values.extend([0.0] * len(message.axes))
        # if len(message.buttons) not in self._num_buttons:
        #     self._num_buttons.append(len(message.buttons))
        #     self._button_states.extend([0.0] * len(message.buttons))
        #     self._button_transitions.extend([0.0] * len(message.buttons))
        print("Num Butt (hehe)", len(message.buttons))
        print("Num Axe", len(message.axes))

        # axes will be listed from controller with least number of axes to most number of axes
        
        for index, axis in enumerate(message.axes, start = axes_offset):
            self._axis_values[index] = axis
        for index, button in enumerate(message.buttons, start = button_offset):
            button_state: bool = bool(button)

            if button_state != self._button_states[index]:
                self._button_transitions[index] += 1
            
            self._button_states[index] = button_state

        # debug
        # debug_message = "Axes: \n"
        # for index, axis in enumerate(self._axis_values):
        #     debug_message += "     "
        #     debug_message += str(index)
        #     debug_message += (". %f\n" % axis)
        # debug_message += "Buttons:\n"
        # for index, button in enumerate(self._button_states):
        #     debug_message += "     "
        #     debug_message += str(index)
        #     debug_message += (". {}".format(button))
        # print(debug_message)

class RovControlState(Enum):
    OFF = 0
    SUSPENDED = 1
    DRIVE_CONTROL = 2
    MANIPULATOR_CONTROL = 3

class RovControl(Node):
    def __init__(self, node_name: str = "rov_control", *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        declare_parameter_dict(self, {
            "update_frequency_hz": 60,
            "mapping": {
                "axis": {
                    # Right Joystick
                    "roll": 0,
                    "pitch": 1,
                    "yaw": 2,
                    "hat_x": 4,
                    "hat_y": 5,
                    # Left Joystick
                    "throttle_y": 6,
                    "throttle_x": 7,
                    "throttle_z": 11,
                    "A1_hat_x": 14,
                    "A1_hat_y": 15,
                    "A1_joy_x": 9,
                    "A1_joy_y": 10,
                    "middle_flippy_thing": 8
                },
                "button": {
                    # Right Joystick
                    "button_1": 0,
                    "button_2": 1,
                    "button_3": 2,
                    "button_4": 3,
                    "button_5": 4,
                    "button_6": 5,
                    "button_7": 6,
                    "button_8": 7,
                    "button_9": 8,
                    "button_10": 9,
                    "button_11": 10,
                    "button_12": 11,
                    "trigger": 0,
                    "grip": 1,
                    "thumb": 1,
                    "stick_top_left": 4,
                    "stick_top_right": 5,
                    "stick_bottom_left": 2,
                    "stick_bottom_right": 3,
                    "base_front_left": 6,
                    "base_front_right": 7,
                    "base_middle_left": 8,
                    "base_middle_right": 9,
                    "base_back_left": 10,
                    "base_back_right": 11,
                    # Left Joystick
                    ""
                    "fire_top": 33,
                    "fire_bottom": 12,
                    "A2": 14,
                    "B1": 15,
                    "A3_up": 17,
                    "A3_right": 18,
                    "A3_down": 19,
                    "A3_left": 20,
                    "A3_center": 21,
                    "A4_up": 22,
                    "A4_right": 23,
                    "A4_down": 24,
                    "A4_left": 25,
                    "A4_center": 26,
                    "C1_up": 27,
                    "C1_right": 28,
                    "C1_down": 29,
                    "C1_left": 30,
                    "D1": 16,
                    "scroll_up": 34,
                    "scroll_down": 35,
                    "En1_up": 34,
                    "En1_down": 35,
                    "left_flippy_up": 36,
                    "left_flippy_down": 37,
                    "Sw1_up": 36,
                    "Sw1_down": 37,
                    "F1": 38,
                    "F2": 39,
                    "F3": 40
                }
            },
            "control": {
                "deadzone": {
                    "roll": 0.1,
                    "pitch": 0.1,
                    "yaw": 0.1,
                    "throttle": 0.1,
                    "throttle_x": 0.1,
                    "throttle_y": 0.1,
                    "throttle_z": 0.1,
                    "throttle_y": 0.1,
                    "throttle_x": 0.1,
                    "throttle_z": 0.1,
                    "A1_hat_x": 0.1,
                    "A1_hat_y": 0.1,
                    "A1_joy_x": 0.1,
                    "A1_joy_y": 0.1,
                    "middle_flippy_thing": 0.1,
                    "hat_x": 0.0,
                    "hat_y": 0.0
                },
                "drive": {
                    "axis": {
                        "vx": "throttle_x",
                        "vy": "throttle_y",
                        "vz": "throttle_z",
                        "omegax": "roll",
                        "omegay": "pitch",
                        "omegaz": "yaw",
                    },
                    "scale": {
                        "vx": 1.0,
                        "vy": -1.0,
                        "vz": 1.0,
                        "omegax": 1.0,
                        "omegay": -1.0,
                        "omegaz": -1.0
                    }
                },
                "manipulator": {
                    "axis": {
                        "wrist": "A1_joy_x",
                        "clamp": "A1_joy_y"
                    },
                    "scale": {
                        "wrist": -1.0,
                        "clamp": -1.0
                    }
                },
                "other": {
                    "lights": {
                        "button": "stick_bottom_right",
                        "pin": 12,
                        "timeout_seconds": 1.0
                    },
                    "mode_switching": {
                        "button": "A2"
                    }
                }
            }
        })

        self._machine = Machine(
            model = self,
            states = RovControlState,
            initial = RovControlState.DRIVE_CONTROL,
            transitions = [
                {
                    "trigger": "change_operating_mode",
                    "source": RovControlState.DRIVE_CONTROL,
                    "dest": RovControlState.MANIPULATOR_CONTROL,
                    "before": self.send_rotation_hold
                },
                {
                    "trigger": "change_operating_mode",
                    "source": RovControlState.MANIPULATOR_CONTROL,
                    "dest": RovControlState.DRIVE_CONTROL
                },
                {
                    # Don't turn on an inactive state when suspending
                    "trigger": "suspend",
                    "source": [
                        RovControlState.OFF,
                        RovControlState.SUSPENDED
                    ],
                    "dest": None
                },
                {
                    "trigger": "suspend",
                    "source": "*",
                    "dest": RovControlState.SUSPENDED,
                    "before": self.on_suspend
                },
                {
                    "trigger": "unsuspend",
                    "source": RovControlState.SUSPENDED,
                    "dest": RovControlState.DRIVE_CONTROL,
                    "before": self.on_unsuspend
                },
                {
                    # Don't panic if not on
                    "trigger": "panic",
                    "source": RovControlState.OFF,
                    "dest": None
                },
                {
                    "trigger": "panic",
                    "source": "*",
                    "dest": RovControlState.OFF,
                    "before": self.on_panic
                }
            ]
        )

        #create timer to run on_update at the frequency defined in "update_frequency_hz"
        self.create_timer(
            1 / self.get_parameter("update_frequency_hz").value,
            self.on_update
        )

        self._joystick = Joystick(
            node = self,
            joystick_topic_name_l = "joy_left",
            joystick_topic_name_r = "joy_right", 
            axis_names = {parameter_name: parameter.value for parameter_name, parameter in self.get_parameters_by_prefix("mapping.axis").items()},
            button_names = {parameter_name: parameter.value for parameter_name, parameter in self.get_parameters_by_prefix("mapping.button").items()}
        )

        #Manipulator control publisher
        self._manipulator_setpoint_publisher = self.create_publisher(
            msg_type = ManipulatorSetpoints,
            topic = "manipulator_setpoints",
            qos_profile = 10
        )

        #Thruster control publisher
        self._thruster_setpoint_publisher = self.create_publisher(
            msg_type = ThrusterSetpoints,
            topic = "thruster_setpoints",
            qos_profile = 10
        )

        #Lights publisher
        self._lights_publisher = self.create_publisher(
            msg_type = PinState,
            topic = "set_rov_gpio",
            qos_profile = 10
        )

        self._estop_subscription = self.create_subscription(
            msg_type = Bool,
            topic = "estop",
            callback = lambda message: self.panic() if message.data else self.suspend(),
            qos_profile = 10
        )

        self._lights_timeout = Timeout(self)

        self._mode_change_acknowledged = False

    def on_panic(self):
        self.get_logger().warn(f"{self.get_name()} has panicked")
        pass

    def on_suspend(self):
        self.get_logger().warn(f"{self.get_name()} is now suspended")
        pass

    def on_unsuspend(self):
        self.get_logger().warn(f"{self.get_name()} is no longer suspended")
        pass

    def on_update(self):
        if self.state == RovControlState.DRIVE_CONTROL:
            self._thruster_setpoint_publisher.publish(
                ThrusterSetpoints(**{
                    thruster_axis: max(-1.0, min(
                        self.get_parameter(f"control.drive.scale.{thruster_axis}").value
                        * self
                            ._joystick.axis(joystick_axis.value)
                            .deadzoned(self.get_parameter(f"control.deadzone.{joystick_axis.value}").value).value,
                    1.0))
                    for thruster_axis, joystick_axis in
                        self.get_parameters_by_prefix("control.drive.axis").items()
                })
            )
            self._manipulator_setpoint_publisher.publish(
                ManipulatorSetpoints(**{
                    manipulator_axis: max(-1.0, min(
                        self.get_parameter(f"control.manipulator.scale.{manipulator_axis}").value
                        * self
                            ._joystick.axis(joystick_axis.value)
                            .deadzoned(self.get_parameter(f"control.deadzone.{joystick_axis.value}").value).value,
                    1.0))
                    for manipulator_axis, joystick_axis in
                        self.get_parameters_by_prefix("control.manipulator.axis").items()
                })
            )
        elif self.state == RovControlState.MANIPULATOR_CONTROL:
            print("HOLY FUDGE  AGHHHH YOU'RE IN MANIPULATOR CONTROL MODE AND SHOULDN'T BE, PRESS A2 OR WHATEVER IT IS")
        #     self._manipulator_setpoint_publisher.publish(
        #         ManipulatorSetpoints(**{
        #             manipulator_axis: max(-1.0, min(
        #                 self.get_parameter(f"control.manipulator.scale.{manipulator_axis}").value
        #                 * self
        #                     ._joystick.axis(joystick_axis.value)
        #                     .deadzoned(self.get_parameter(f"control.deadzone.{joystick_axis.value}").value).value,
        #             1.0))
        #             for manipulator_axis, joystick_axis in
        #                 self.get_parameters_by_prefix("control.manipulator.axis").items()
        #         })
        #     )
        elif self.state == RovControlState.OFF :
            #send all manipulators zero command
            self._manipulator_setpoint_publisher.publish(
                ManipulatorSetpoints(**{
                    manipulator_axis: 0.0
                for manipulator_axis, joystick_axis in
                    self.get_parameters_by_prefix("control.manipulator.axis").items()
                })
            )
            #set all thrusters to zero
            self._thruster_setpoint_publisher.publish(
                ThrusterSetpoints(**{
                    thruster_axis: 0.0
                    for thruster_axis, joystick_axis in
                        self.get_parameters_by_prefix("control.drive.axis").items()
                })
            )
            #turn off the light
            self._lights_publisher.publish(
                PinState(
                    pin = self.get_parameter("control.other.lights.pin").value,
                    state = False
                )
            )
            #shut down the node (this assumes that the estop has been pressed and we do not want it to reenable)
            rclpy.shutdown()
            return
        
        #check light_toggle_button and toggle the lights on or off accordingly
        light_toggle_button = self._joystick.button(self.get_parameter("control.other.lights.button").value)
        if self._lights_timeout.passed() and light_toggle_button:
            self._lights_publisher.publish(
                PinState(
                    pin = self.get_parameter("control.other.lights.pin").value,
                    state = light_toggle_button.toggled
                )
            )
            self._lights_timeout = Timeout(
                node = self,
                seconds = self.get_parameter("control.other.lights.timeout_seconds").value
            )
        
        #if the mode_change_button is pressed, trigger change_operating_mode to switch what is being controlled
        mode_change_button = self._joystick.button(self.get_parameter("control.other.mode_switching.button").value)
        if mode_change_button and not self._mode_change_acknowledged:
            self._mode_change_acknowledged = True
            self.change_operating_mode()
        elif not mode_change_button and self._mode_change_acknowledged:
            self._mode_change_acknowledged = False

    def send_rotation_hold(self):
        self._thruster_setpoint_publisher.publish(ThrusterSetpoints())

def main():
    rclpy.init()
    rclpy.spin(RovControl())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
