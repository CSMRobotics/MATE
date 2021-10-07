#include "Component.hpp"
#include "libs/bno055/bno055.h"

class IMU : public Component{

public:
    IMU();
    ~IMU();

    void Update();
    void AutoUpdate();
    void Stop();

private:
    

};

