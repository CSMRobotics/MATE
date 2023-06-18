#include "manipulator_controller/manipulator_controller.hpp"
#include <algorithm>

namespace {
    void declare_manipulator_controller_parameters(rclcpp::Node*const node) {
        node->declare_parameter("wrist_invert", false);
        node->declare_parameter("wrist_lower_bound", 0.0);
        node->declare_parameter("wrist_upper_bound", 180.0);
        node->declare_parameter("wrist_pwm_pin", 14);
        node->declare_parameter("clamp_invert", false);
        node->declare_parameter("clamp_lower_bound", 0.0);
        node->declare_parameter("clamp_upper_bound", 180.0);
        node->declare_parameter("clamp_pwm_pin", 11);
    }

    void clamp(float* toClamp, double lower, double upper) {
        *toClamp = std::min(*toClamp, static_cast<float>(upper));
        *toClamp = std::max(*toClamp, static_cast<float>(lower));
    }
}

ManipulatorController::ManipulatorController() : Node(std::string("manipulator_controller")) {
    declare_manipulator_controller_parameters(this);

    manip_setpoints = this->create_subscription<rov_interfaces::msg::ManipulatorSetpoints>("manipulator_setpoints", 10, std::bind(&ManipulatorController::setpoint_callback, this, std::placeholders::_1));
    pwm_pub = this->create_publisher<rov_interfaces::msg::PWM>("PWM", 10);
}

void ManipulatorController::setpoint_callback(const rov_interfaces::msg::ManipulatorSetpoints::SharedPtr msg) {
    // map values from rov_control
    float wrist = (msg->wrist + 1) * 90 * (this->get_parameter("wrist_invert").as_bool() ? -1 : 1); // map [-1, 1] to [0, 180]
    float clamp = (msg->clamp + 1) * 90 * (this->get_parameter("clamp_invert").as_bool() ? -1 : 1); // map [-1, 1] to [0, 180]

    // clamp the received values
    ::clamp(&wrist, this->get_parameter("wrist_lower_bound").as_double(), this->get_parameter("wrist_upper_bound").as_double());
    ::clamp(&clamp, this->get_parameter("clamp_lower_bound").as_double(), this->get_parameter("clamp_upper_bound").as_double());

    // publish PWM
    // wrist pwm
    auto pwm = rov_interfaces::msg::PWM();
    pwm.angle_or_throttle = wrist;
    pwm.channel = this->get_parameter("wrist_pwm_pin").as_int();
    pwm_pub->publish(pwm);

    // clamp pwm
    pwm = rov_interfaces::msg::PWM();
    pwm.angle_or_throttle = clamp;
    pwm.channel = this->get_parameter("clamp_pwm_pin").as_int();
    pwm_pub->publish(pwm);
}