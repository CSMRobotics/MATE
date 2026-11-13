#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "SubControls.h"
#include "Thruster.hpp"
#include "throttleController.h"

#include <Eigen/Geometry>
#include <array>

class FlightController {
public:
    /**
     * @brief Construct a new Flight Controller object
     * 
     * @param updatePeriod: How often the update loop will be called in seconds
     * @param init_depth: current depth at time of initialization in meters world frame NED
     * @param init_orientation: current orientation at time of initialization in world frame NED
     * @param orient_kp, orient_ki, orient_kd: orientation stabilization PID constants
     * @param depth_kp, depth_ki, depth_kd: depth stabilization PID constants
     * @param max_omega: Max rotation velocity in rad/s
     * @param max_vertical_vel: Max vertical translational velocity m/s
     * @param throttle_controller: Initialized instance of throttle_controller
     */
	FlightController(
        float update_period, 
        float init_depth,
        Eigen::Quaternionf& init_orientation,
        float orient_kp, float orient_ki, float orient_kd,
        float depth_kp, float depth_ki, float depth_kd,
        float max_omega,
        float max_vertical_vel,
        ThrottleController* throttle_controller
    );

    //***************************************************************************************
    // Flight loop, call these functions in order
    // MUST BE CALLED IN A TIMED LOOP at all times

	/**
	 * @brief Updates the sub's current orientation state in world frame (NED, North East Down)
     *      World->Body rotation
     * 
     * @param orientation: orientation of the submarine in world frame
	 */
	void update_orient_state(const Eigen::Quaternionf& orientation);

    /**
     * @brief Updates sub's current depth in world frame (meters)
     */
    void update_depth_state(float depth);

    /**
     * @brief Update submarine controls received from Flight Computer
     * 
     * @param controls: Control setpoints
     */
    void update_sub_controls(const SubControls& controls);

    /**
     * @brief The main flight control loop using PID control
     *  Update sub's thrusters based on IMU data and setpoints
     * 
     * @param angular_velocity: angular velocity sub in FRD body frame (rad/s)
     */
    void update_pid(const Eigen::Vector3f& angular_velocity);

    // End of main flight loop functions
    //***************************************************************************************
    /**
     * @brief The main flight control loop in very stupid mode.
     *  Updates sub's thrusters purely based on SubControls controls
     *  Use either update_simple() or update_PID(), not both
     */
    void update_simple();

    /**
	 * @brief Change the target orientation in world frame (NED, North East Down)
     *      World->Body rotation.
     * 
     * @param orientation: orientation of the submarine in world frame
	 */
	void set_orient_setpoint(const Eigen::Quaternionf& orientation);

    /**
     * @brief Updates target depth in world frame (NED, meters)
     */
    void set_depth_setpoint(float depth);

    /**
     * @brief Change PID constants used for orientation stabilization
     */
    void set_orient_pid_constants(float kp, float ki, float kd);

    /**
     * @brief Change PID constants used for depth stabilization
     */
    void set_depth_pid_constants(float kp, float ki, float kd);

    /**
     * @brief Tell thrusters to stop immediately, will not allow any thrust until disabled.
     */
    void activate_safe_mode();
    void disable_safe_mode();

    /**
     * @brief Sets submarine's target depth to its current depth given by update_depth_state()
     */
    void tare_depth();

private:
    /*************************************************************************************** */
    // Math constants
    float _update_period;    // dt
    float _update_freq;     // 1/dt
    float _max_omega;       // rad/s
    float _max_vertical_vel; // m/s
    float _orient_kp, _orient_ki, _orient_kd;
    float _depth_kp, _depth_ki, _depth_kd;
    const Eigen::Vector3f _orient_integral_error_limit = Eigen::Vector3f(3.0f, 3.0f, 3.0f);
    const float _depth_integral_error_limit = 5.0f;

    /*************************************************************************************** */
    // Flight Controller State
    bool _safe_mode = true;
    bool _init_update_loop = true;  // Is this the initial run of the flight control update loop?

    Eigen::Quaternionf _orient_state = Eigen::Quaternionf::Identity();     // Measured orientation state in NED coords
    Eigen::Quaternionf _orient_setpoint = Eigen::Quaternionf::Identity();   // Desired orientation in NED coords
    Eigen::Quaternionf _prev_orient_setpoint = Eigen::Quaternionf::Identity();

    float _depth_state = 0;         // Measured depth in meters
    float _prev_depth_state = 0;
    float _depth_setpoint = 0;      // Desired depth in meters

    // Rotation errors
    Eigen::Vector3f _orient_error = Eigen::Vector3f::Zero();      // rads
    Eigen::Vector3f _prev_orient_error = Eigen::Vector3f::Zero();
    Eigen::Vector3f _orient_int_error = Eigen::Vector3f::Zero();   // rads * s
    Eigen::Vector3f _orient_der_error = Eigen::Vector3f::Zero();   // rads/s

    // Depth errors
    float _depth_error = 0;      // m
    float _depth_int_error = 0;  // m*s
    float _depth_der_error = 0;  // m/s

    // Robot movement setpoints
    Eigen::Vector3f _translate_setpoints = Eigen::Vector3f::Zero();    // <x, y, z> FRD
    Eigen::Vector3f _attitude_setpoints = Eigen::Vector3f::Zero();     // <roll, pitch, yaw> FRD

    /**
     * @brief Updates internal state of the FlightController
     *      Called every run of the flight control loop
     */
    void _update_state();

    /*************************************************************************************** */
    // Thrusters
    std::array<Thruster, NUM_THRUSTERS> _thrusters;
    ThrottleController* _throttle_controller;

    /**
     * @brief Update thrusters based on _translate_setpoints and _attitude_setpoints
     */
    void _update_thruster_actuations();

    /**
     * @brief Clamps actuations proportionally to the largest abs(throttle) exceeding [-1, 1]
     * 
     * @param throttles 
     */
    void _clamp_actuations(Eigen::Matrix<float,NUM_THRUSTERS,1>* throttles);

    /*************************************************************************************** */
    // Control
    SubControls _control_setpoint = SubControls{0, 0, 0, 0, 0, 0};
};

#endif