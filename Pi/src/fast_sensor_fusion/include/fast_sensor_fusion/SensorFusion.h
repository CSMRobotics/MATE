#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <rclcpp/rclcpp.hpp>

#include "MavlinkPubSub.hpp"
#include "DataStructs.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"

constexpr int64_t LOOP_PERIOD_NS = 1'000'000; // 1 ms
static constexpr const char* SHM_NAME = "/mavlink_shm_sensor_fusion";

/**
 * @brief Performs state estimation based on sensor information from MAVLink sensor data
 * 
 */
class SensorFusion : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    SensorFusion();

    /**
     * @brief The "main" loop.
     * 
     */
    void update();

    /**
     * @brief Make a guess
     */
    void shutdown();
    
    /**
     * @brief Get the state of the robot
     * 
     */
    void get_state();

private:
    /**
     * @brief gets the latest IMU data from the buffer and stores in data structures
     *        Discards existing messages in buffer, gets the latest one
     * @return Returns 1 when new data is received, 0 when no data is available
     */
    int _update_imu_data();

    /**
     * @brief Updates orientation state with a Mahony Filter
     * 
     */
    void _update_orientation();

    /**
     * @brief Close shm
     * 
     * @return Returns 0 on succes, -1 on error
     */
    void _close_shm();

    // Private data members
    // IMU Data
    IMU9Axis _imu_9;
    IMU9Axis _prev_imu_9;
    IMU6Axis _imu_6;

    // Rotation state
    Eigen::Quaterniond _curr_quad = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond _prev_quad;
    Eigen::Vector3d _integral_error;
    double _dt; // seconds

    // PI Constants NEEDS TESTING
    const double Kp = 0.5;
    const double Ki = 0.02;

    // MAVLink Shared Memory
    MavlinkPubSub* _mav_sub;
};

#endif // SENSOR_FUSION_H