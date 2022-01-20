#include "IMU.hpp"

IMU::IMU() {
    m_BNO.begin(); // connect to BNO055
    usleep(1000 * 100); // wait 100 ms to get some fusion data in registers
    m_NDOF_Data.fullyCalibrated = m_BNO.isFullyCalibrated(); // check for calibration
}

IMU::~IMU() {
    // nothing special to do upon destruction
}

void IMU::Update() {
    // update all NDOF data
    m_NDOF_Data.orientation = m_BNO.getQuat();
    m_NDOF_Data.accelerometer = m_BNO.getVector(Vector_Type::ACCELEROMETER);
    m_NDOF_Data.magnetometer = m_BNO.getVector(Vector_Type::MAGNETOMETER);
    m_NDOF_Data.gyroscope = m_BNO.getVector(Vector_Type::GYROSCOPE);
    m_NDOF_Data.euler = m_BNO.getVector(Vector_Type::EULER);
    m_NDOF_Data.linearaccel = m_BNO.getVector(Vector_Type::LINEARACCEL);
    m_NDOF_Data.gravity = m_BNO.getVector(Vector_Type::GRAVITY);
}

void IMU::AutoUpdate() {
    Update();
}

void IMU::Stop() {
    // nothing special to do upon E-Stop
}

void IMU::Calibrate() {
    while(!m_NDOF_Data.fullyCalibrated) {
        m_NDOF_Data.fullyCalibrated = m_BNO.isFullyCalibrated();
    }
}