#include "FlightController.h"

FlightController::FlightController(
    float update_period, 
    float altitude,
    float kp, 
    float ki, 
    float kd, 
    ThrottleController* throttle_controller
) {
    // Initialize variables
    _update_period = update_period;
    _update_freq = 1/update_period;
    _altitude_state = altitude;
    _safe_mode = true;
    _kp = kp;
    _ki = ki;
    _kd = kd;

    _thrusters = load_thrusters();
    _throttle_controller = throttle_controller;
}

void FlightController::update_orient_state(const Eigen::Quaternionf& orientation) {
    _orient_state = orientation;
}

void FlightController::update_altitude_state(float altitude) {
    _altitude_state = altitude;
}

void FlightController::update_sub_controls(const SubControls& controls) {
    _control_setpoint = controls;

    // Update desired rotation setpoint
    float delta_yaw = (_control_setpoint.omegaZ * MAX_OMEGA) * _update_period;
    Eigen::Quaternionf delta_q = Eigen::Quaternionf::Identity();
    delta_q = Eigen::Quaternionf(Eigen::AngleAxisf(delta_yaw, Eigen::Vector3f::UnitZ()));
    _orient_setpoint = _orient_setpoint * delta_q;
    _orient_setpoint.normalize();

    // Update desired altitude setpoint
    float delta_alt = controls.velocityZ * _update_period;
    _altitude_setpoint += delta_alt;
}

void FlightController::update_pid(const Eigen::Vector3f& angular_velocity) {
    // Compute orientation error
    Eigen::Quaternionf q_error = _orient_setpoint * _orient_state.conjugate();
    if (q_error.w() < 0) {
        q_error.coeffs() *= -1.0f;
    }
    q_error.normalize();

    // Get orientation error vector (points in axis of rotation, magnitude is rotation in radians)
    float vec_norm = q_error.vec().norm();
    _prev_orient_error = _orient_error;
    if (vec_norm < 1e-6) {
        _orient_error = 2.0f * q_error.vec();
    } else {
        float angle = Eigen::AngleAxisf(q_error).angle();
        _orient_error = (angle/vec_norm) * q_error.vec();
    }

    // Rotate angular_velocity in world frame
    Eigen::Vector3f angular_velocity_world = _orient_state.inverse().toRotationMatrix() * angular_velocity;

    // PID that error
    _orient_int_error += _orient_error * _update_period;
    _orient_int_error = _orient_int_error.cwiseMax(-_orient_int_error_lim).cwiseMin(_orient_int_error_lim);
    _orient_der_error = angular_velocity_world;
    
    Eigen::Vector3f torque = _kp * _orient_error + _ki * _orient_int_error - _kd * _orient_der_error;
    
    // Compute corretive thruster setpoints
    
}

void FlightController::update_simple() {
    // Update translation and attitude setpoints
    _translate_setpoints = {_control_setpoint.velocityX, _control_setpoint.velocityY, _control_setpoint.velocityZ};
    _attitude_setpoints = {_control_setpoint.omegaX, _control_setpoint.omegaY, _control_setpoint.omegaZ};

    // Send to thrusterController
    _update_thruster_actuations();
}

void FlightController::_update_thruster_actuations() {
    if (_safe_mode) {
        float controller_actuations[NUM_THRUSTERS] = {0};
        _throttle_controller->transmit(controller_actuations);
        return;
    }

    // Get actuations for each thruster
    Eigen::Matrix<float, NUM_THRUSTERS, 1> unnormalized_actuations = Eigen::Matrix<float, NUM_THRUSTERS, 1>::Zero();
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        Thruster t = _thrusters[i];
        // Actuation = Linear Actuation(0) + Rotational Actuation (0)
        unnormalized_actuations(i,0) = t.thrust.dot(_translate_setpoints)
            + t.position.cross(t.thrust).dot(_attitude_setpoints);
    }
    
    // scale actuations by abs of largest actuation request
    Eigen::Matrix<float, NUM_THRUSTERS, 1> actuations = unnormalized_actuations;
    _clamp_actuations(&actuations);

    // apply the actuations to the thrusters
    float controller_actuations[NUM_THRUSTERS];
    for(int i = 0; i < NUM_THRUSTERS; i++) {
        int64_t thruster_idx = _thrusters.at(i).controller_idx;
        controller_actuations[thruster_idx] = actuations(i, 0);
    }
    _throttle_controller->transmit(controller_actuations);
}

void FlightController::_clamp_actuations(Eigen::Matrix<float,NUM_THRUSTERS,1>* throttles) {
    Eigen::Index loc;
    throttles->minCoeff(&loc);
    double ratioA = std::abs(-1.0 / std::min((*throttles)(loc), -1.0f));
    throttles->maxCoeff(&loc);
    double ratioB = std::abs(1.0 / std::max((*throttles)(loc), 1.0f));
    if(ratioA < ratioB) {
        *throttles = *throttles * ratioA;
    } else {
        *throttles = *throttles * ratioB;
    }
}