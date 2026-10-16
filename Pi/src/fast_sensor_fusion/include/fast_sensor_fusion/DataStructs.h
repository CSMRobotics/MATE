#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <cstdint>
/**
 * @brief 9 Axis IMU data
 * 
 */
struct IMU9Axis {
    uint64_t time_usec = 0;

    double gyroX = 0;   // rad/s
    double gyroY = 0;
    double gyroZ = 0;

    double accelX = 0;  // m/s^2
    double accelY = 0;
    double accelZ = 0;

    double magX = 0;    // gauss
    double magY = 0;
    double magZ = 0;
};

/**
 * @brief 6 Axis IMU data
 * 
 */
struct IMU6Axis {
    uint64_t time_usec = 0;

    double gyroX = 0;   // rad/s
    double gyroY = 0;
    double gyroZ = 0;

    double accelX = 0;  // m/s^2
    double accelY = 0;
    double accelZ = 0;
};

/**
 * @brief Orientation state Quaternion
 * 
 */
struct Quaternion {
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

struct Vector3 {
    double x = 0;
    double y = 0;
    double z = 0;
};

#endif // DATA_STRUCTS_H