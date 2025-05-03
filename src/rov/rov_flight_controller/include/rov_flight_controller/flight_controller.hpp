#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include "eigen3/Eigen/Core"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/empty.hpp>

#include "Thruster.hpp"
#include "rov_interfaces/msg/bno055_data.hpp"
#include "rov_interfaces/msg/thruster_setpoints.hpp"
#include "rov_interfaces/msg/bar30_data.hpp"
#include "rov_interfaces/msg/tsys01_data.hpp"
#include "rov_interfaces/msg/pwm.hpp"

#include "rov_interfaces/srv/create_continuous_servo.hpp"

#define NUM_THRUSTERS 8
#define UPDATE_MS 1000/60

// TODO: ensure services are thread-safe
class FlightController : public rclcpp::Node {
public:
    FlightController();
    ~FlightController();

private:
    void registerThrusters();
    void toggle_PID(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response);
    void setpoint_callback(const rov_interfaces::msg::ThrusterSetpoints::SharedPtr setpoints);
    void bno_callback(const rov_interfaces::msg::BNO055Data::SharedPtr bno_data);
    void bar_callback(const rov_interfaces::msg::Bar30Data::SharedPtr bar_data);
    void tsys_callback(const rov_interfaces::msg::TSYS01Data::SharedPtr tsys_data);
    void updateNone();
    void updateSimple();
    void updatePID() {}; // disabled for now
    void clampthrottles(Eigen::Matrix<double,NUM_THRUSTERS,1>* throttles);

    void safe(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response);
    void desafe(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response);

    void updateDepth(Eigen::Matrix<double, 6, 1>& desired_forces_torques);
    void depthTare(const std_srvs::srv::Empty_Request::SharedPtr request, std_srvs::srv::Empty_Response::SharedPtr response);

    void updatePitchRoll(Eigen::Matrix<double, 6, 1>& desired_forces_torques);
    void updateStab();

    rclcpp::Subscription<rov_interfaces::msg::ThrusterSetpoints>::SharedPtr thruster_setpoint_subscription;
    rclcpp::Subscription<rov_interfaces::msg::BNO055Data>::SharedPtr bno_data_subscription;
    rclcpp::Subscription<rov_interfaces::msg::TSYS01Data>::SharedPtr tsys_data_subscription;
    rclcpp::Subscription<rov_interfaces::msg::Bar30Data>::SharedPtr bar_data_subscription;
    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr pwm_publisher;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr toggle_PID_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr depth_tare_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr safe_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr desafe_service;
    rclcpp::CallbackGroup::SharedPtr pca9685_registration_callbackgroup;
    rclcpp::Client<rov_interfaces::srv::CreateContinuousServo>::SharedPtr pca9685_client;
    std::array<rclcpp::Client<rov_interfaces::srv::CreateContinuousServo>::SharedFutureWithRequest, NUM_THRUSTERS> pca9685_requests;

    std::function<void(void)> _update_simple = std::bind(&FlightController::updateSimple, this);
    std::function<void(void)> _update_pid = std::bind(&FlightController::updatePID, this);
    std::function<void(void)> _update_stab = std::bind(&FlightController::updateStab, this);
    rclcpp::TimerBase::SharedPtr control_loop;

    std::chrono::time_point<std::chrono::high_resolution_clock> last_updated;

    std::mutex stall_mutex;

    rov_interfaces::msg::BNO055Data::SharedPtr bno_data;
    std::mutex bno_mutex;
    rov_interfaces::msg::TSYS01Data::SharedPtr tsys_data;
    std::mutex tsys_mutex;
    rov_interfaces::msg::Bar30Data::SharedPtr bar30_data;
    std::mutex bar30_mutex;

    Eigen::Vector3d translation_setpoints = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d attitude_setpoints = Eigen::Vector3d(0,0,0);

    float tare_depth;
    float depth_setpoint;
    bool keep_depth = false;
    bool free_orientation = false;

    std::mutex setpoint_mutex;
    Eigen::Quaterniond quaternion_reference;
    Eigen::Vector3d linear_accel_last = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d linear_integral = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d linear_velocity = Eigen::Vector3d(0,0,0);

    std::chrono::time_point<std::chrono::high_resolution_clock> startUpdateLoopTime;

    std::array<Thruster, NUM_THRUSTERS> thrusters;
    Eigen::Matrix<double, 6, NUM_THRUSTERS> thruster_geometry;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thruster_geometry_pseudo_inverse;
    Eigen::DiagonalMatrix<double, NUM_THRUSTERS> thruster_coefficient_matrix;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thruster_coefficient_matrix_times_geometry;
    std::unordered_map<int, int> thruster_index_to_PWM_pin;
};

#endif