#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>

#include "flight_controller/pid.hpp"
#include "rov_interfaces/msg/thruster_setpoints.hpp"
#include "rov_interfaces/msg/pwm.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rov_interfaces/srv/create_continuous_servo.hpp"

#define NUM_THRUSTERS 8

class FlightController : public rclcpp::Node {
public:
    FlightController();
private:
    void updatePID();
    void updateJoy();

    PID position_pid;
    PID velocity_pid;

    rclcpp::TimerBase::SharedPtr pid_update_timer;
    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr pwm_publisher;
    std::array<std::shared_future<rov_interfaces::srv::CreateContinuousServo::Response::SharedPtr>, NUM_THRUSTERS> pca9685_response_futures;
    rclcpp::CallbackGroup::SharedPtr pca9685_callback_group;
};

#endif