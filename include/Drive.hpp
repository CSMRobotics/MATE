#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "Component.hpp"
#include "Joystick.hpp"
#include "IMU.hpp"
#include "ServoDriver.hpp"

#include "CSMUtil.hpp"

#include <chrono>

#define THRUSTER_NUM 8

enum DriveState {
    MANUAL = 0,
    PID_ATTITUDE = 1,
    AUTO = 2,
};

enum ModeState {
    TRANSLATE = 0,
    ROTATE = 1,
};

class Thruster {
    public:
        Thruster(int num, ServoDriver* driver);
        
        float calculateThrustAtPoint(float throttle);
        void set(float throttle);
    protected:

    private:
        int num;
        float throttle;
        ServoDriver* driver;
        csmutil::Vector3f thrusterPosition;
        csmutil::Vector3f thrusterThrust;
};

class Drive : public Component {
public:
    Drive() = default;
    Drive(ServoDriver* driver, Joystick* joystick, IMU* imu);
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

    csmutil::Quaternionf createRef(Axes axes);

    uint64_t timeCurrentMillis;
    uint64_t timePreviousMillis;
    uint64_t dt;

    ServoDriver* driver;
    csmutil::NonLinearQuaternionController pid;
    csmutil::Vector3f torque;
    csmutil::Quaternionf q_ref;
    Joystick* joystick;
    Axes axes;
    IMU* imu;

    bool activeComponent = false;

    DriveState state = MANUAL;
    ModeState modeState = TRANSLATE;
};

#endif // DRIVE_HPP