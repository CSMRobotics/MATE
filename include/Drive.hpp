#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "Component.hpp"
#include "Thruster.hpp"
#include "Joystick.hpp"
#include "IMU.hpp"

#include <chrono>

#define THRUSTER_NUM 8

class Drive : public Component {
public:
    Drive() = default;
    Drive(Joystick* joystick, IMU* imu, Thruster* thrusters[]);
    ~Drive();

    void Update();
    void AutoUpdate();
    void Stop();
protected:

private:
    uint64_t millis() {
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
        return ms;
    }

    uint64_t timeInitialMillis = millis();
    uint64_t timeCurrentMillis = timeInitialMillis;
    uint64_t timePreviousMillis = timeInitialMillis;

    Thruster* thrusters[THRUSTER_NUM];
    Joystick* joystick;
    IMU* imu;
};

#endif // DRIVE_HPP