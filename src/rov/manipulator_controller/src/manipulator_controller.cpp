#include "manipulator_controller/manipulator_controller.hpp"

ManipulatorController::ManipulatorController() : Node(std::string("manipulator_controller")) {
    manip_setpoints = this->create_subscription<rov_interfaces::msg::ManipulatorSetpoints>("manipulator_setpoints", 10, std::bind(&ManipulatorController::setpoint_callback, this, std::placeholders::_1));
    pwm_pub = this->create_publisher<rov_interfaces::msg::PWM>("PWM", 10);
}

void ManipulatorController::setpoint_callback(const rov_interfaces::msg::ManipulatorSetpoints::SharedPtr msg) {
    // do stuff
}