#ifndef IMU_HPP
#define IMU_HPP

#include "Component.hpp"
#include "CSMBNO055.hpp"
#include "Vector3.hpp"
#include <iostream>

class IMU : public Component{

public:
    IMU();
    ~IMU();

    void Update();
    void AutoUpdate();
    void Stop();

private:
    BNO055 m_BNO;
};

#endif // IMU_HPP