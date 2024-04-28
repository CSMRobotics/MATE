#include "rewrite.hpp"

FlightController::FlightController() : rclcpp::Node("flight_controller") {
    // Initialize all matrices
    Thruster_Coefficients_Matrix = {40, 40, 40, 40, 40, 40, 40, 40}; // TODO: tune
    updateConfiguration();
    updateModel();

    // TODO: initialize PWM publisher
}

void FlightController::updateAllSimple(float dt) {
    updatePID(dt);
    applyUpdatedPID();
}

void FlightController::applyUpdatedPID() {
    doControlAllocation();

    // TODO: apply allocated controls
}

void FlightController::updateConfiguration() {
    // update the configuration matrix with the thruster info array
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        // Top three orientation, bottom three torque
        Configuration_Matrix.col(i) << Thruster_Info.at(i).ori, Thruster_Info.at(i).pos.cross(Thruster_Info.at(i).ori);
    }

    // A^+ = A^T * (A*A^T)^-1
    Configuration_Matrix_PseudoInverse = Configuration_Matrix.transpose() * (Configuration_Matrix * Configuration_Matrix.transpose()).inverse();
}

void FlightController::updateModel() {
    // Thruster Model

    // Buoyancy and Gravity
    // TODO: assumes symmetrical about xz and xy planes
    Gravitational_And_Buoyancy_Force << 2 * (Buoyancy - Weight) * (Orientation.x() * Orientation.z() - Orientation.y() * Orientation.w()),
                                        2 * (Buoyancy - Weight) * (Orientation.y() * Orientation.z() - Orientation.x() * Orientation.w()),
                                        (Weight - Buoyancy) * (2*powf(Orientation.x(), 2) + 2*powf(Orientation.y(), 2) - 1),
                                        Cg_Position.z() * Weight * 2 * (Orientation.y() * Orientation.z() + Orientation.x() * Orientation.w()),
                                        Cg_Position.z() * Weight * 2 * (Orientation.x() * Orientation.z() + Orientation.y() * Orientation.w()),
                                        0;
}

void FlightController::doControlAllocation() {
    // u = K^-1 * T^+ * t
    Control_Inputs = Inv_Thruster_Coefficients_Matrix * Configuration_Matrix_PseudoInverse * Control_Forces;
}

void FlightController::updatePID(float dt) {
    static Eigen::Vector<float, 7> eta;
    eta << Position, quat2Euler(Orientation);
    Eigen::Vector<float, 6> PID_Error = eulerToTransformation(quat2Euler(Orientation)) * (Desired_Postion_Orientation - eta);

    // P
    Control_Forces = PID_Gains.col(0) * PID_Error;

    // I
    PID_Error_Integral += PID_Gains.col(1) * PID_Error * dt;
    // TODO: saturate
    Control_Forces += PID_Error_Integral;

    // D
    Control_Forces += PID_Gains.col(2) * (PID_Error - PID_Error_Last) / dt;

    PID_Error_Last = PID_Error;
}

Eigen::Matrix<float, 7, 6> FlightController::quaternionToTransformation(Eigen::Quaternionf quat)
{
    return Eigen::Matrix<float, 7, 6>();
}

Eigen::Matrix<float, 6, 6> eulerToTransformation(Eigen::Vector<float, 3> euler) {
    Eigen::Matrix<float, 3, 3> R;
    float phi = euler(0);
    float theta = euler(1);
    float psi = euler(2);

    R << cosf(psi)*cosf(theta), -sinf(psi)*cosf(phi)+cosf(psi)*sinf(theta)*sinf(phi), sinf(psi)*sinf(phi)+cosf(psi)*cosf(phi)*cosf(theta),
         sin(psi)*cosf(theta), cosf(psi)*cosf(phi)+sinf(phi)*sinf(theta)*sinf(psi), -cosf(psi)*sinf(phi)+sinf(theta)*sinf(psi)*cosf(phi),
         -sinf(theta), cosf(theta)*sinf(phi), cosf(theta)*cosf(phi);

    Eigen::Matrix<float, 3, 3> T;
    T << 1, sinf(phi)*tanf(theta), cosf(phi)*cosf(theta),
         0, cosf(phi), -sinf(phi),
         0, sinf(phi)/cosf(theta), cosf(phi)/cosf(theta);

    Eigen::Matrix3f Z;

    Eigen::Matrix<float, 6, 6> J;

    J << R, Z, 
    Z, T;

    return J;
}

Eigen::Vector3f quat2Euler(Eigen::Quaternionf quat) {
    float q0 = quat(0);
    float q1 = quat(1);
    float q2 = quat(2);
    float q3 = quat(3);

    float roll;
    float pitch;
    float yaw;

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = std::sqrt(1 + 2 * (q0 * q2 - q1 * q3));
    float cosp = std::sqrt(1 - 2 * (q0 * q2 - q1 * q3));
    pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    return Eigen::Vector3f (roll, pitch, yaw);
}