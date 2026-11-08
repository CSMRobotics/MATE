#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include "SubControls.h"

#include <Eigen/Geometry>

class FlightController {
public:
    /**
     * @brief Construct a new Flight Controller object
     * 
     * @param updatePeriod: How often the update loop will be called in seconds
     * @param altitude: altitude in meters
     * @param kp, ki, kd: PID constants
     */
	FlightController(float update_period, float altitude, float kp, float ki, float kd);

    //***************************************************************************************
    // Flight loop, call these functions in order

	/**
	 * @brief Updates the sub's current orientation state in world frame (NED, North East Down)
     *      World->Body rotation
     * 
     * @param orientation: orientation of the submarine in world frame
	 */
	void update_orient_state(const Eigen::Quaternionf& orientation);

    /**
     * @brief Updates sub's current altitude in world frame (meters)
     */
    void update_altitude_state(float altitude);

    /**
     * @brief Update submarine controls received from User or Autonomous mode
     * 
     * @param controls: Control setpoints
     */
    void update_sub_controls(const SubControls& controls);

    /**
     * @brief The main flight control loop using PID control
     *  Update sub's thrusters based on IMU data
     * 
     * @param angular_velocity: angular velocity sub in FRD body frame (rad/s)
     */
    void update_pid(const Eigen::Vector3f& angular_velocity);

    // End of main flight loop functions
    //***************************************************************************************
    /**
     * @brief The main flight control loop in very stupid mode.
     *  Updates sub's thrusters purely based on User controls
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
     * @brief Updates target altitude in world frame (meters)
     */
    void set_altitude_setpoint(float altitude);

    /**
     * @brief Change PID constants used for stabilization
     */
    void set_pid_constants(float kp, float ki, float kd);

    /**
     * @brief Tell thrusters to stop immediately, will not allow any thrust until disabled.
     */
    void activate_safe_mode();
    void disable_safe_mode();

    /**
     * @brief Sets submarine's target altitude to its current altitude
     */
    void tare_altitude();

private:
    // Math constants
    float _update_period;    // dt
    float _update_freq;     // 1/dt
    float _kp, _ki, _kd;
    const Eigen::Vector3f _orient_int_error_lim = Eigen::Vector3f(3.0f, 3.0f, 3.0f);

    // States
    bool _safe_mode = true;  
    Eigen::Quaternionf _orient_state = Eigen::Quaternionf::Identity();     // Measured orientation state
    Eigen::Quaternionf _orient_setpoint = Eigen::Quaternionf::Identity();   // Desired orientation
    float _altitude_state = 0;         // Measured altitude in meters
    float _altitude_setpoint = 0;      // Desired altitude in meters

    // Rotation errors
    Eigen::Vector3f _orient_error = Eigen::Vector3f(0, 0, 0);      // Radians
    Eigen::Vector3f _prev_orient_error = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f _orient_int_error = Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f _orient_der_error = Eigen::Vector3f(0, 0, 0);

    // Robot movement setpoints
    Eigen::Vector3f _translate_setpoints = Eigen::Vector3f(0, 0, 0);    // [-1, 1] <x, y, z>
    Eigen::Vector3f _attitude_setpoints = Eigen::Vector3f(0, 0, 0);     // [-1, 1] <roll, pitch, yaw>

    SubControls _control_setpoint = SubControls{0, 0, 0, 0, 0, 0};
};

#endif