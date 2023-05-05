#include "pca9685/PCA9685_Node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

PCA9685_Node::PCA9685_Node() : Node(std::string("pca9685")) {
    _pwm_subscription = this->create_subscription<rov_interfaces::msg::PWM>(std::string("pwm"), 10, 
        std::bind(&PCA9685_Node::topic_callback, this, _1));
    _create_continuous_servo_service = this->create_service<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo", 
        std::bind(&PCA9685_Node::create_continuous_servo, this, _1, _2));
    _create_servo_service = this->create_service<rov_interfaces::srv::CreateServo>("create_servo", 
        std::bind(&PCA9685_Node::create_servo, this, _1, _2));
}

void PCA9685_Node::topic_callback(const rov_interfaces::msg::PWM::SharedPtr msg) {
#if DEBUG_OUTPUT
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "PCA9685 set channel %i to %f", msg->channel, msg->angle_or_throttle);
#endif
    servoDriver.setOutput(msg->channel, msg->angle_or_throttle);
}

void PCA9685_Node::create_continuous_servo(const std::shared_ptr<rov_interfaces::srv::CreateContinuousServo_Request> request, 
        std::shared_ptr<rov_interfaces::srv::CreateContinuousServo_Response> response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PCA9685 Received Continuous Servo Creation Request on Channel %i", request->channel);
    servoDriver.addContinuousServo(request->channel);
    response->result = true;
    response->channel = request->channel;
}

void PCA9685_Node::create_servo(const std::shared_ptr<rov_interfaces::srv::CreateServo_Request> request, 
        std::shared_ptr<rov_interfaces::srv::CreateServo_Response> response) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "PCA9685 Received Servo Creation Request on Channel %i", request->channel);
    servoDriver.addServo(request->channel);
    response->result = true;
    response->channel = request->channel;
}