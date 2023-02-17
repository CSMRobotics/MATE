#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include "eigen3/Eigen/Core"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/empty.hpp>

#include "Thruster.hpp"
#include "rov_interfaces/msg/bno055_data.hpp"
#include "rov_interfaces/msg/thruster_setpoints.hpp"
#include "rov_interfaces/msg/pwm.hpp"

#define NUM_THRUSTERS 8

// TODO: ensure services are thread-safe
class FlightController : public rclcpp::Node {
public:
    FlightController();

private:
    void registerThrusters();
    void toggle_PID(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response);
    void setpoint_callback(const rov_interfaces::msg::ThrusterSetpoints::SharedPtr setpoints);
    void bno_callback(const rov_interfaces::msg::BNO055Data::SharedPtr bno_data);
    void updateSimple();
    void updatePID();
    void normalizethrottles(Eigen::Matrix<double,NUM_THRUSTERS,1>* throttles);

    rclcpp::Subscription<rov_interfaces::msg::ThrusterSetpoints>::SharedPtr thruster_setpoint_subscription;
    rclcpp::Subscription<rov_interfaces::msg::BNO055Data>::SharedPtr bno_data_subscription;
    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr _publisher;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_PID_service;

    std::function<void(void)> _update = std::bind(&FlightController::updateSimple, this);
    std::function<void(void)> _update2 = std::bind(&FlightController::updatePID, this);

    rclcpp::TimerBase::SharedPtr pid_control_loop;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_updated;

    std::mutex stall_mutex;

    rov_interfaces::msg::BNO055Data bno_data;
    std::mutex bno_mutex;
    Eigen::Vector3d translation_setpoints = Eigen::Vector3d(3,1);
    Eigen::Vector3d attitude_setpoints = Eigen::Vector3d(3,1);
    std::mutex setpoint_mutex;
    Eigen::Quaterniond quaternion_reference;
    Eigen::Vector3d linear_accel_last;
    Eigen::Vector3d linear_integral;
    Eigen::Vector3d linear_velocity_err = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d linear_velocity_err_last;

    std::array<Thruster, NUM_THRUSTERS> thrusters;
    Eigen::Matrix<double, 6, NUM_THRUSTERS> thruster_geometry;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thruster_geometry_pseudo_inverse;
    Eigen::DiagonalMatrix<double, NUM_THRUSTERS> thruster_coefficient_matrix;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thruster_coefficient_matrix_times_geometry;
    std::unordered_map<int, int> thruster_index_to_PWM_pin;

    double Pq = 1.0, Pw = 1.0; // TODO: tune these gain constants
    double kp = 1.0, ki = 1.0, kd = 1.0; // TODO: ROS2 Params for real-time tuning (no more 30000 compilations wooooooooo)
    double pc_1 = 1.0, pc_2 = 1.0;
};

#endif