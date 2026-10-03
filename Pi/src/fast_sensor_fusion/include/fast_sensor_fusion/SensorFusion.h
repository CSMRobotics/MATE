#include <rclcpp/rclcpp.hpp>

#include "SharedMavlink.h"

/**
 * @brief BNO086 sensor data structure
 * 
 */
struct BNO086Data {
    double gyroX;
    double gyroY;
    double gyroZ;

    double accelX;
    double accelY;
    double accelZ;

    double magX;
    double magY;
    double magZ;
};

class SensorFusion : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    SensorFusion();
    
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
    BNO086Data _bno_data;

    // MAVLink Shared Memory
    int _shm_fd;
    SharedMavlink* _mav_shm;
    uint64_t _shm_read_idx;
};