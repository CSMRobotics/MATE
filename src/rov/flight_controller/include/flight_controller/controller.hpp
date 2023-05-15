#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <rclcpp/node.hpp>
#include <std_srvs/srv/empty.hpp>

#include <eigen3/Eigen/Core>

#include <flight_controller/Thruster.hpp>
#include <csm_common_interfaces/msg/e_stop.hpp>
#include <rov_interfaces/msg/bar02_data.hpp>
#include <rov_interfaces/msg/bno055_data.hpp>
#include <rov_interfaces/msg/thruster_setpoints.hpp>
#include <rov_interfaces/msg/pwm.hpp>

#include <mutex>
#include <unordered_map>

#define NUM_THRUSTERS 8
#define UPDATE_RATE_HZ 60
#define NS_TO_S 1e-6

struct bar_data {
    bar_data() = default;
    bar_data(rov_interfaces::msg::Bar02Data::SharedPtr msg);

    _Float32 altitude;
    _Float32 depth;
    _Float32 pressure;
    _Float32 temperature_c;
};

struct bno_data {
    bno_data() = default;
    bno_data(rov_interfaces::msg::BNO055Data::SharedPtr msg);

    Eigen::Quaterniond quaternion;
    Eigen::Vector3d accelerometer;
    Eigen::Vector3d magnetometer;
    Eigen::Vector3d gyroscope;
    Eigen::Vector3d euler;
    Eigen::Vector3d linearaccel;
    Eigen::Vector3d gravity;
    int8_t temp;
    uint8_t calibration;
};

struct setpoint_data {
    setpoint_data() = default;
    setpoint_data(rov_interfaces::msg::ThrusterSetpoints::SharedPtr msg);

    Eigen::Vector3d translational;
    Eigen::Vector3d rotational;
};

class FlightController : public rclcpp::Node {
public:
    FlightController();

private:
    void update_callback();

    void update_none([[maybe_unused]] long dt_ns) {};
    void update_simple(long dt_ns);
    void update_linear_PID(long dt_ns);
    void update_non_linear_PID(long dt_ns);

    void update_coriolis_centripetal_matrix(const Eigen::Vector3d& v, const Eigen::Vector3d& w);
    void update_hydrodynamic_damping_matrix(const Eigen::Vector3d& d, const Eigen::Vector3d& v, const Eigen::Vector3d& w);
    void update_gravitational_and_buoyancy_vector(const Eigen::Vector3d& euler);
    void update_gravitational_and_buoyancy_vector(const Eigen::Quaterniond& quat);

    void estop_callback(csm_common_interfaces::msg::EStop::SharedPtr msg);
    void estop_handler(bool is_estop_call);
    void restart_after_estop(const std::shared_ptr<std_srvs::srv::Empty_Request> req, const std::shared_ptr<std_srvs::srv::Empty_Response> res);

    void setpoint_callback(rov_interfaces::msg::ThrusterSetpoints::SharedPtr msg);
    void bar_callback(rov_interfaces::msg::Bar02Data::SharedPtr msg);
    void bno_callback(rov_interfaces::msg::BNO055Data::SharedPtr msg);

    rclcpp::Subscription<rov_interfaces::msg::Bar02Data>::SharedPtr bar_sub;
    rclcpp::Subscription<rov_interfaces::msg::BNO055Data>::SharedPtr bno_sub;
    rclcpp::Subscription<rov_interfaces::msg::ThrusterSetpoints>::SharedPtr setpoint_sub;
    rclcpp::Subscription<csm_common_interfaces::msg::EStop>::SharedPtr estop_sub;
    bool ESTOP = false;

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_after_estop_service;

    rclcpp::Publisher<rov_interfaces::msg::PWM>::SharedPtr pwm_pub;
    std::mutex pwm_mutex;

    rclcpp::TimerBase::SharedPtr update_timer;
    std::function<void(long)> desired_update = std::bind(&FlightController::update_simple, this, std::placeholders::_1);
    std::chrono::time_point<std::chrono::high_resolution_clock> last_updated;
    std::mutex update_mutex;

    Eigen::Matrix<double, 6, NUM_THRUSTERS> thruster_geometry;
    Eigen::Matrix<double, NUM_THRUSTERS, 6> thruster_geometry_left_pseudoinverse;
    std::unordered_map<int, int> thruster_idx_to_pwm_pin;
    std::array<Thruster, NUM_THRUSTERS> thrusters;
    Eigen::Matrix<double, NUM_THRUSTERS, 1> actuations = Eigen::Matrix<double, NUM_THRUSTERS, 1>::Zero();
    Eigen::Vector3d desired_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d desired_moment = Eigen::Vector3d::Zero();

    Eigen::Matrix3d R_bton; // Transformation Matrix of linear velocity from body frame to world
    Eigen::Matrix<double, 4, 3> Tq; // Transformation of angular velocity from body frame to world
    Eigen::Matrix<double, 6, 1> eta; // Pose
    Eigen::Matrix<double, 6, 1> nu; // Velocity
    Eigen::Matrix3d I; // Inertia moments about body frame
    Eigen::Matrix<double, 6, 6> C_rb; // Coriolis and centripetal matrix
    Eigen::Matrix<double, 6, 6> M_rb; // Rigid-body mass matrix
    Eigen::Matrix<double, 6, 6> D; // hydrodynamic damping matrix
    Eigen::Vector<double, 6> g_res; // gravitational and buoyancy force vector

    setpoint_data setpoints;
    bar_data pressure_data;
    bno_data imu_data;
    std::mutex setpoint_mutex;
    std::mutex bar_mutex;
    std::mutex bno_mutex;
    
};

#endif