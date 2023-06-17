#ifndef MANIPULATOR_CONTROLLER_HPP
#define MANIPULATOR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/msg/manipulator_setpoints.hpp"

namespace {

}

class ManipulatorController : public rclcpp::Node {
public:
    ManipulatorController();

private:
    void setpoint_callback(const rov_interfaces::msg::ManipulatorSetpoints::SharedPtr msg);

    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr pwm_pub;
    rclcpp::Subscription<rov_interfaces::msg::ManipulatorSetpoints>::SharedPtr manip_setpoints;
};

#endif