#ifndef MANIPULATOR_CONTROLLER_HPP
#define MANIPULATOR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/msg/manipulator_setpoints.hpp"
#include "rov_interfaces/srv/create_servo.hpp"

#include <unordered_map>

class ManipulatorController : public rclcpp::Node {
public:
    ManipulatorController();

private:
    void setpoint_callback(const rov_interfaces::msg::ManipulatorSetpoints::SharedPtr msg);
    void register_servos();

    rclcpp::CallbackGroup::SharedPtr pca9685_registration_callbackgroup;
    rclcpp::Client<rov_interfaces::srv::CreateServo>::SharedPtr pca9685_client;
    std::array<rclcpp::Client<rov_interfaces::srv::CreateServo>::SharedFutureWithRequest, 2> pca9685_requests;

    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr pwm_pub;
    rclcpp::Subscription<rov_interfaces::msg::ManipulatorSetpoints>::SharedPtr manip_setpoints;
};

#endif