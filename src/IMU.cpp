#include "IMU.hpp"

IMU::IMU() {
    m_BNO.begin();
}

IMU::~IMU() {

}

void IMU::Update() {
    std::cout << m_BNO.getVector(Vector_Type::EULER) << std::endl;
}

void IMU::AutoUpdate() {

}

void IMU::Stop() {

}