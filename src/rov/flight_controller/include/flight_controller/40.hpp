#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <functional>

#include <rclcpp/rclcpp.hpp>

#include <eigen3/Eigen/Eigen>

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
    void updateSetpoints(rov_interfaces::msg::ThrusterSetpoints::ConstSharedPtr msg);

    PID<Eigen::Vector<float, 6>> position_pid;
    PID<Eigen::Vector<float, 6>> velocity_pid;

    rclcpp::TimerBase::SharedPtr pid_update_timer;
    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr pwm_publisher;
    rclcpp::Subscription<rov_interfaces::msg::ThrusterSetpoints>::SharedPtr setpoint_subscriber;
    std::array<std::shared_future<rov_interfaces::srv::CreateContinuousServo::Response::SharedPtr>, NUM_THRUSTERS> pca9685_response_futures;
    std::array<int64_t, NUM_THRUSTERS> pca9685_request_ids;
    rclcpp::CallbackGroup::SharedPtr pca9685_callback_group;

    std::map<uint64_t, int> pwm_pin_lookup;
    std::map<uint64_t, bool> pwm_pin_ready;

    std::function<float(float)> thrust_to_pwm_function;

    Eigen::Vector<float, 6> pos_setpoint;
    Eigen::Vector<float, 6> pos_measurement;
    Eigen::Vector<float, 6> vel_measurement;

    Eigen::Matrix<float, 6, NUM_THRUSTERS> G;
    Eigen::FullPivHouseholderQR<Eigen::Matrix<float, 6, NUM_THRUSTERS>> G_qr_decomp;
    Eigen::Vector<float, 6> F;
    Eigen::Vector<float, NUM_THRUSTERS> T;
    Eigen::Vector<float, NUM_THRUSTERS> U;
};

#endif