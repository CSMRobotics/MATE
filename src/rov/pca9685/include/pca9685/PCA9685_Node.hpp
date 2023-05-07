#ifndef PCA9685_NODE_HPP
#define PCA9685_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/srv/create_continuous_servo.hpp"
#include "rov_interfaces/srv/create_servo.hpp"

#include "pca9685/ServoDriver.hpp"
#include "JHPWMPCA9685/JHPWMPCA9685.h"

class PCA9685_Node : public rclcpp::Node {
public:
    PCA9685_Node();
private:
    void topic_callback(const rov_interfaces::msg::PWM::SharedPtr);
    void create_continuous_servo(const rov_interfaces::srv::CreateContinuousServo_Request::SharedPtr request, rov_interfaces::srv::CreateContinuousServo_Response::SharedPtr response);
    void create_servo(const rov_interfaces::srv::CreateServo_Request::SharedPtr request, rov_interfaces::srv::CreateServo_Response::SharedPtr response);

    ServoDriver servoDriver;
    rclcpp::Subscription<rov_interfaces::msg::PWM>::SharedPtr _pwm_subscription;
    rclcpp::Service<rov_interfaces::srv::CreateContinuousServo>::SharedPtr _create_continuous_servo_service;
    rclcpp::Service<rov_interfaces::srv::CreateServo>::SharedPtr _create_servo_service;
};

#endif