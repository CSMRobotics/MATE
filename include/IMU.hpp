#ifndef IMU_HPP
#define IMU_HPP

#include "Component.hpp"
#include "CSMBNO055.hpp"
#include "Vector3.hpp"
#include <iostream>

struct NDOF_Data {
    csmutil::Quaterniond orientation;
    csmutil::Vector3d accelerometer;
    csmutil::Vector3d magnetometer;
    csmutil::Vector3d gyroscope;
    csmutil::Vector3d euler;
    csmutil::Vector3d linearaccel;
    csmutil::Vector3d gravity;
    bool fullyCalibrated = false;
};

class IMU : public Component {
public:
    IMU();
    ~IMU();

    void Update();
    void AutoUpdate();
    void Stop();

    // warning: blocking if not calibrated
    void Calibrate();

    NDOF_Data m_NDOF_Data;
private:
    BNO055 m_BNO;
};

#endif // IMU_HPP