#include "rclcpp/rclcpp.hpp"

#include "rov_interfaces/msg/pwm.hpp"
#include "rov_interfaces/srv/create_continuous_servo.hpp"
#include "rov_interfaces/srv/create_servo.hpp"

#include "JHPWMPCA9685.h"
#include "ServoDriver.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class PCA9685_Node : public rclcpp::Node {
public:
    PCA9685_Node() : Node(std::string("pca9685")) {
        _pwm_subscription = this->create_subscription<rov_interfaces::msg::PWM>(std::string("pwm"), 10, 
            std::bind(&PCA9685_Node::topic_callback, this, _1));
        _create_continuous_servo_service = this->create_service<rov_interfaces::srv::CreateContinuousServo>("create_continuous_servo", 
            std::bind(&PCA9685_Node::create_continuous_servo, this, _1, _2));
        _create_servo_service = this->create_service<rov_interfaces::srv::CreateServo>("create_servo", 
            std::bind(&PCA9685_Node::create_servo, this, _1, _2));
    }

private:
    void topic_callback(const rov_interfaces::msg::PWM::SharedPtr msg) {
        if(msg->is_continuous_servo) {
            servoDriver.setThrottle(msg->channel, msg->angle_or_throttle);
        } else {
            servoDriver.setAngle(msg->channel, msg->angle_or_throttle);
        }
    }

    void create_continuous_servo(const std::shared_ptr<rov_interfaces::srv::CreateContinuousServo_Request> request, 
            std::shared_ptr<rov_interfaces::srv::CreateContinuousServo_Response> response) {
        servoDriver.addContinuousServo(request->channel);
        response->result = true;
        response->channel = request->channel;
    }

    void create_servo(const std::shared_ptr<rov_interfaces::srv::CreateServo_Request> request, 
            std::shared_ptr<rov_interfaces::srv::CreateServo_Response> response) {
        servoDriver.addServo(request->channel);
        response->result = true;
        response->channel = request->channel;
    }

    ServoDriver servoDriver;
    rclcpp::Subscription<rov_interfaces::msg::PWM>::SharedPtr _pwm_subscription;
    rclcpp::Service<rov_interfaces::srv::CreateContinuousServo>::SharedPtr _create_continuous_servo_service;
    rclcpp::Service<rov_interfaces::srv::CreateServo>::SharedPtr _create_servo_service;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA9685_Node>());
    rclcpp::shutdown();
    return 0;
}