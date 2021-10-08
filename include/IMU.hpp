#ifndef IMU_HPP
#define IMU_HPP

#include "Component.hpp"
#include "bno055.h"
#include <iostream>

class IMU : public Component{

public:
    IMU();
    ~IMU();

    void Update();
    void AutoUpdate();
    void Stop();

private:
    

};

#endif // IMU_HPP