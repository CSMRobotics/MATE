#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <rclcpp/rclcpp.hpp>

#include "SharedMavlink.h"

constexpr int64_t LOOP_PERIOD_NS = 1'000'000; // 1 ms

/**
 * @brief 9 Axis IMU data
 * 
 */
struct IMU9Axis {
    uint64_t time_usec;

    double gyroX;   // rad/s
    double gyroY;
    double gyroZ;

    double accelX;  // m/s^2
    double accelY;
    double accelZ;

    double magX;    // gauss
    double magY;
    double magZ;
};

/**
 * @brief 6 Axis IMU data
 * 
 */
struct IMU6Axis {
    uint64_t time_usec;

    double gyroX;   // rad/s
    double gyroY;
    double gyroZ;

    double accelX;  // m/s^2
    double accelY;
    double accelZ;
};

/**
 * @brief Orientation state Quaternion
 * 
 */
struct OrientationQuad {
    double x;
    double y;
    double z;
    double w;
};

struct Vector3 {
    double x;
    double y;
    double z;
};

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
     * @brief Opens the named pipe for MAVLink
     * 
     * @return Returns 0 on succes, -1 on error
     */
    int _setup_shm();

    /**
     * @brief Close shm
     * 
     * @return Returns 0 on succes, -1 on error
     */
    int _close_shm();

    // Private data members
    // IMU Data
    IMU9Axis _imu_9;
    IMU9Axis _prev_imu_9;
    IMU6Axis _imu_6;

    // Rotation state
    OrientationQuad _curr_quad;
    OrientationQuad _prev_quad;

    // MAVLink Shared Memory
    int _shm_fd;
    SharedMavlink* _mav_shm;
    uint64_t _shm_read_idx;
};

#endif // SENSOR_FUSION_H