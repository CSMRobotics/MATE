#include "fast_sensor_fusion/SensorFusion.h"
#include "mavlink/common/mavlink.h"

#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <cstdint>
#include <cmath>

SensorFusion::SensorFusion() : rclcpp::Node("fast_sensor_fusion") {
    // Setup shared memory
    try {
        _mav_sub = new MavlinkPubSub(false, SHM_NAME);
    } catch (const std::system_error& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup Shared Memory: %s", e.what());
        exit(-1);
    }
}

void SensorFusion::update() {
    // Get time to delay for next loop iteration
    timespec start_time{};
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    // The main loop
    if (this->_update_imu_data()) {
        _update_orientation();
    }

    // Delay until next call
    // Advance by exactly one period.
    timespec next_time = start_time;
    next_time.tv_nsec += LOOP_PERIOD_NS;

    while (next_time.tv_nsec >= 1'000'000'000) {
        next_time.tv_nsec -= 1'000'000'000;
        ++next_time.tv_sec;
    }

    // Sleep until the next time
    clock_nanosleep(
        CLOCK_MONOTONIC,
        TIMER_ABSTIME,
        &next_time,
        nullptr);
}

void SensorFusion::shutdown() {
    _close_shm();
}

//****************************************************** */
// Sensor Fusion
void SensorFusion::_update_orientation() {
    // Average IMU data
    IMU9Axis imu;
    imu.accelX = (_imu_6.accelX + _imu_9.accelX) / 2;
    imu.accelY = (_imu_6.accelY + _imu_9.accelY) / 2;
    imu.accelZ = (_imu_6.accelZ + _imu_9.accelZ) / 2;

    imu.gyroX = (_imu_6.gyroX + _imu_9.gyroX) / 2;
    imu.gyroY = (_imu_6.gyroY + _imu_9.gyroY) / 2;
    imu.gyroZ = (_imu_6.gyroZ + _imu_9.gyroZ) / 2;

    imu.magX = _imu_9.magX;
    imu.magY = _imu_9.magY;
    imu.magZ = _imu_9.magZ;

    // Normalize acceleration and magnetometer
    Eigen::Vector3d norm_imu_accel(imu.accelX, imu.accelY, imu.accelZ);
    norm_imu_accel.normalize();
    
    Eigen::Vector3d norm_imu_mag(imu.magX, imu.magY, imu.magZ);
    norm_imu_mag.normalize();

    // Compute Gravity error
    // Estimated gravity vector
    Eigen::Vector3d pred_accel;
    pred_accel.x() = 2 * (_curr_quad.x() * _curr_quad.z() - _curr_quad.w() * _curr_quad.y());
    pred_accel.y() = 2 * (_curr_quad.w() * _curr_quad.x() + _curr_quad.y() * _curr_quad.z());
    pred_accel.z() = 
        (_curr_quad.w() * _curr_quad.w()) -
        (_curr_quad.x() * _curr_quad.x()) -
        (_curr_quad.y() * _curr_quad.y()) +
        (_curr_quad.z() * _curr_quad.z());
    pred_accel.normalize();

    // Cross product predicted and actual for gravity error
    Eigen::Vector3d grav_error = norm_imu_accel.cross(pred_accel);

    // Compute Magenetometer error
    Eigen::Matrix3d orient_mtx = _curr_quad.toRotationMatrix();
    Eigen::Vector3d pred_mag = orient_mtx.transpose() * Eigen::Vector3d(1, 0, 0); // Assumes North is positive 1 on X
    Eigen::Vector3d mag_error = norm_imu_mag.cross(pred_mag);

    // Compute total error with weights bc mag is less reliable
    const double mag_weight = 0.2;
    Eigen::Vector3d total_error =
        grav_error +
        mag_weight * mag_error;

    // Proportional, Integral (PI) correction
    _integral_error += total_error * _dt;
    Eigen::Vector3d gyro(
        imu.gyroX,
        imu.gyroY,
        imu.gyroZ);

    gyro += Kp * total_error
      + Ki * _integral_error;

    // Compute quaternion derivative
    Eigen::Quaterniond omega_q(
        0.0,
        gyro.x(),
        gyro.y(),
        gyro.z());
    Eigen::Quaterniond q_dot =_curr_quad * omega_q;
    q_dot.coeffs() *= 0.5;

    // Update orientation
    _curr_quad.coeffs() += q_dot.coeffs() * _dt;
    _curr_quad.normalize();

    std::cout << _curr_quad << std::endl;
}

//****************************************************** */
// IMU data shared memory management

void SensorFusion::_close_shm() {
    delete _mav_sub;
}

int SensorFusion::_update_imu_data() {
    mavlink_message_t message{};
    mavlink_status_t status{};

    if (_mav_sub->read(message, status) != 1) {
        return 0;
    }

    if (message.msgid == MAVLINK_MSG_ID_RAW_IMU) {
        mavlink_raw_imu_t raw_imu{};

        mavlink_msg_raw_imu_decode(
            &message,
            &raw_imu);

        std::cout
            << "  IMU ID = " << static_cast<double>(raw_imu.id) << "\n"
            << "  time_usec = " << static_cast<double>(raw_imu.time_usec) << "\n"
            << "  xacc = " << static_cast<double>(raw_imu.xacc) << "\n"
            << "  yacc = " << static_cast<double>(raw_imu.yacc) << "\n"
            << "  zacc = " << static_cast<double>(raw_imu.zacc) << "\n"
            << "  xgyro = " << static_cast<double>(raw_imu.xgyro) << "\n"
            << "  ygyro = " << static_cast<double>(raw_imu.ygyro) << "\n"
            << "  zgyro = " << static_cast<double>(raw_imu.zgyro) << "\n"
            << "  xmag = " << static_cast<double>(raw_imu.xmag) << "\n"
            << "  ymag = " << static_cast<double>(raw_imu.ymag) << "\n"
            << "  zmag = " << static_cast<double>(raw_imu.zmag) << "\n"
            << "  temperature = " << static_cast<double>(raw_imu.temperature)
            << std::endl;

        // Update IMU data (FAKE DATA FOR NOW)
        _prev_imu_9 = _imu_9;

        _imu_9.time_usec = raw_imu.time_usec;

        _imu_9.accelX = raw_imu.xacc;
        _imu_9.accelY = raw_imu.yacc;
        _imu_9.accelZ = raw_imu.zacc;

        _imu_6.accelX = raw_imu.xacc;
        _imu_6.accelY = raw_imu.yacc;
        _imu_6.accelZ = raw_imu.zacc;

        _imu_9.gyroX = raw_imu.xgyro;
        _imu_9.gyroY = raw_imu.ygyro;
        _imu_9.gyroZ = raw_imu.zgyro;

        _imu_6.gyroX = raw_imu.xgyro;
        _imu_6.gyroY = raw_imu.ygyro;
        _imu_6.gyroZ = raw_imu.zgyro;

        _imu_9.magX = raw_imu.xmag;
        _imu_9.magY = raw_imu.ymag;
        _imu_9.magZ = raw_imu.zmag;

        // Update dt
        _dt = (_imu_9.time_usec - _prev_imu_9.time_usec) / 1e6;
    }

    return 1;
}