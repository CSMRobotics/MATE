#ifndef THRUSTER_HPP
#define THRUSER_HPP

#include "eigen3/Eigen/Dense"

struct Thruster {
    Thruster();
    Thruster(Eigen::Vector3d position, Eigen::Vector3d thrust, int pwm_pin);
    Eigen::Vector3d position;
    Eigen::Vector3d thrust;
    int pwm_pin;
};

#endif //THRUSTER_HPP