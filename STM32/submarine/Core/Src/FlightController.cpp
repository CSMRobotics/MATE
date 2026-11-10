#include "FlightController.h"

#include <algorithm>

FlightController::FlightController(
    float update_period, 
    float init_depth,
    Eigen::Quaternionf& init_orientation,
    float orient_kp, float orient_ki, float orient_kd, 
    float depth_kp, float depth_ki, float depth_kd,
    float max_omega,
    float max_vertical_vel,
    ThrottleController* throttle_controller
) {
    // Initialize math constants
    _update_period = update_period;
    _update_freq = 1/update_period;
    
    _safe_mode = true;
    _orient_kp = orient_kp;
    _orient_ki = orient_ki;
    _orient_kd = orient_kd;
    _depth_kp = depth_kp;
    _depth_ki = depth_ki;
    _depth_kd = depth_kd;
    _max_omega = max_omega;
    _max_vertical_vel = max_vertical_vel;

    // Initialize state and setpoints
    _safe_mode = true;
    _init_update_loop = true;

    _orient_state = init_orientation;
    _orient_setpoint = init_orientation;
    _prev_orient_setpoint = init_orientation;

    _depth_state = init_depth;
    _prev_depth_state = init_depth;
    _depth_setpoint = init_depth;

    _thrusters = load_thrusters();
    _throttle_controller = throttle_controller;
}

void FlightController::update_orient_state(const Eigen::Quaternionf& orientation) {
    _orient_state = orientation;
}

void FlightController::update_depth_state(float depth) {
    _prev_depth_state = _depth_state;
    _depth_state = depth;
}

void FlightController::update_sub_controls(const SubControls& controls) {
    _control_setpoint = controls;

    // Update desired rotation setpoint
    _prev_orient_setpoint = _orient_setpoint;
    float delta_yaw = (_control_setpoint.omegaZ * _max_omega) * _update_period;
    Eigen::Quaternionf delta_q = Eigen::Quaternionf::Identity();
    delta_q = Eigen::Quaternionf(Eigen::AngleAxisf(delta_yaw, Eigen::Vector3f::UnitZ()));
    _orient_setpoint = _orient_setpoint * delta_q;
    _orient_setpoint.normalize();

    // Update desired depth setpoint
    float delta_depth = (controls.velocityZ * _max_vertical_vel) * _update_period;
    _depth_setpoint += delta_depth;
}

void FlightController::update_pid(const Eigen::Vector3f& angular_velocity) {
    // Compute orientation error
    Eigen::Quaternionf q_error = _orient_state.conjugate() * _orient_setpoint;
    if (q_error.w() < 0.0f) {
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

    // Rotate angular_velocity into world frame
    Eigen::Vector3f angular_velocity_ned = _orient_state.inverse().toRotationMatrix() * angular_velocity;

    // Feedforward rate of change of target orientation
    Eigen::Quaternionf q_setpoint_delta = _prev_orient_setpoint.conjugate() * _orient_setpoint;
    if (q_setpoint_delta.w() < 0.0f) {
        q_setpoint_delta.coeffs() *= -1.0f;
    }
    q_setpoint_delta.normalize();

    Eigen::AngleAxisf delta_rotation(q_setpoint_delta);
    Eigen::Vector3f angular_velocity_desired_ned = Eigen::Vector3f::Zero();

    if (delta_rotation.angle() > 1e-6f) {
        angular_velocity_desired_ned = delta_rotation.axis() * delta_rotation.angle() / _update_period;
    }

    // PID that error
    _orient_int_error += _orient_error * _update_period;
    _orient_int_error = _orient_int_error.cwiseMax(-_orient_integral_error_limit).cwiseMin(_orient_integral_error_limit);
    _orient_der_error = angular_velocity_desired_ned - angular_velocity_ned;
    
    Eigen::Vector3f attitude_setpoints = 
        _orient_kp * _orient_error + 
        _orient_ki * _orient_int_error +
        _orient_kd * _orient_der_error;
    
    // Rotate to body frame (FRD)
    attitude_setpoints = _orient_state.toRotationMatrix() * attitude_setpoints;

    // Update target attitude
    _attitude_setpoints = attitude_setpoints;

    // Depth PID calculation
    _depth_error = _depth_setpoint - _depth_state;
    _depth_int_error += _depth_error * _update_period;
    _depth_int_error = std::max(-_depth_integral_error_limit, std::min(_depth_int_error, _depth_integral_error_limit));
    _depth_der_error = (_depth_state - _prev_depth_state) * _update_freq;

    float translate_z_setpoint = 
        _depth_kp * _depth_error + 
        _depth_ki * _depth_int_error -
        _depth_kd * _depth_der_error;

    // Rotate to body frame
    Eigen::Vector3f translate_setpoints_ned = {0, 0, translate_z_setpoint};
    Eigen::Vector3f translate_setpoints_frd = _orient_state.toRotationMatrix() * translate_setpoints_ned;

    // Update translation vector
    Eigen::Vector3f control_translate_setpoints_frd = {
        _control_setpoint.velocityX,    // FRD
        _control_setpoint.velocityY,    // FRD
        0
    };
    _translate_setpoints = translate_setpoints_frd + control_translate_setpoints_frd;

    _update_thruster_actuations();

    // Update FlightController state
    _update_state();
}

void FlightController::update_simple() {
    // Update translation and attitude setpoints
    _translate_setpoints = {_control_setpoint.velocityX, _control_setpoint.velocityY, _control_setpoint.velocityZ};
    _attitude_setpoints = {_control_setpoint.omegaX, _control_setpoint.omegaY, _control_setpoint.omegaZ};

    // Send to thrusterController
    _update_thruster_actuations();

    // Update FlightController state
    _update_state();
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
    float controller_actuations[NUM_THRUSTERS] = {0};
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

void FlightController::set_orient_setpoint(const Eigen::Quaternionf& orientation) {
    _prev_orient_setpoint = _orient_setpoint;
    _orient_setpoint = orientation;
}

void FlightController::set_depth_setpoint(const float depth) {
    _depth_setpoint = depth;
}

void FlightController::set_orient_pid_constants(float kp, float ki, float kd) {
    _orient_kp = kp;
    _orient_ki = ki;
    _orient_kd = kd;
}

void FlightController::set_depth_pid_constants(float kp, float ki, float kd) {
    _depth_kp = kp;
    _depth_ki = ki;
    _depth_kd = kd;
}

void FlightController::activate_safe_mode() {
    _safe_mode = true;
}

void FlightController::disable_safe_mode() {
    // Reset all state
    _init_update_loop = true;
    _safe_mode = false;
}

void FlightController::_update_state() {
    if (_init_update_loop) {
        _init_update_loop = false;
    }
}