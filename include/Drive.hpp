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

    uint64_t m_timeCurrentMillis;
    uint64_t m_timePreviousMillis;
    uint64_t m_dt;

    ServoDriver* m_driver;
    csmutil::NonLinearQuaternionController m_attitudePID;
    csmutil::LinearPIDController m_throttlePID;
    csmutil::Vector3d m_desiredTorque;
    csmutil::Vector3d m_desiredForce;
    csmutil::Quaterniond m_q_ref;
    csmutil::Quaterniond m_orientation;
    csmutil::Quaterniond m_orientationLast;
    csmutil::Vector3d m_linaccel;
    csmutil::Vector3d m_angaccel;
    Joystick* m_joystick;
    IMU* m_imu;

    bool activeComponent = false;

    DriveState state = MANUAL;
    ModeState modeState = TRANSLATE;
};

#endif // DRIVE_HPP