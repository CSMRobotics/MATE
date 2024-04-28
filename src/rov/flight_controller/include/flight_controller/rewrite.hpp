#ifndef FLIGHT_CONTROLLER_HPP
#define FLIGHT_CONTROLLER_HPP

#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/empty.hpp>

#include <eigen3/Eigen/Core>

#include <flight_controller/Thruster.hpp>
#include <csm_common_interfaces/msg/e_stop.hpp>
#include <rov_interfaces/msg/pwm.hpp>

#define NUM_THRUSTERS 8

struct Thruster {
    Eigen::Vector3f pos;
    Eigen::Vector3f ori;
};

class FlightController : public rclcpp::Node {
public:
    FlightController();
    void updateAllSimple(float dt);
private:
    void applyUpdatedPID();
    void updateConfiguration();
    void updateModel();
    void doControlAllocation();
    void updatePID(float dt);
    Eigen::Matrix<float, 7, 6> quaternionToTransformation(Eigen::Quaternionf quat);
    Eigen::Matrix<float, 6, 6> eulerToTransformation(Eigen::Vector<float, 3> euler);

    Eigen::Vector3f quat2Euler(Eigen::Quaternionf quat);

    float mass = 1.0f; // mass of ROV kg
    float grav = 9.81f; // local gravitational acceleration m/s^2
    float displaced_vol = 1.0f; // volume of fluid displaced m^3
    uint16_t rho = 1000; // density of water kg/m^3

    const float Weight = mass * grav;
    const float Buoyancy = rho * grav * displaced_vol;

    Eigen::Vector<float, 6> Gravitational_And_Buoyancy_Force;
    
    Eigen::Vector3f Cg_Position;
    Eigen::Vector3f Position;
    Eigen::Quaternionf Orientation;
    Eigen::Vector3f Velocity_b;
    
    Eigen::Vector<float, 6> Desired_Postion_Orientation;

    std::array<Thruster, NUM_THRUSTERS> Thruster_Info;
    Eigen::DiagonalMatrix<float, NUM_THRUSTERS> Thruster_Coefficients_Matrix;
    Eigen::DiagonalMatrix<float, NUM_THRUSTERS> Inv_Thruster_Coefficients_Matrix;
    Eigen::Matrix<float, 6, NUM_THRUSTERS> Configuration_Matrix;
    Eigen::Matrix<float, 6, NUM_THRUSTERS> Configuration_Matrix_PseudoInverse;
    Eigen::Vector<float, 6> Control_Forces;
    Eigen::Vector<float, NUM_THRUSTERS> Control_Inputs;

    Eigen::Matrix<float, 6, 3> PID_Gains;
    Eigen::Vector<float, 6> PID_Error_Last;
    Eigen::Vector<float, 6> PID_Error_Integral;
};

#endif