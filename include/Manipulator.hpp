#ifndef MANIPULATOR_HPP
#define MANIPULATOR_HPP

#include "Component.hpp"
#include "Joystick.hpp"
#include "ServoDriver.hpp"
#include <iostream>

// Create basic class definition responsible for moving the robot
class Manipulator : public Component{
public:
    Manipulator() = default;
    Manipulator(Joystick* joystick, ServoDriver* driver);
    ~Manipulator() = default;

    void Update();
    void AutoUpdate();
    void Stop();
private:
    unsigned char m_elbowServo = 8;
    unsigned char m_elbowServo2 = 9;
    unsigned char m_levelServo = 10;
    unsigned char m_wristServo = 11;
    unsigned char m_clampServo = 12;
    
    unsigned char m_clampButton = 0;
    unsigned char m_chickenButton = 1;
    
    Joystick* joystick;
    ServoDriver* driver;

    bool m_isChicken = false;
    bool m_isClamping = false;
    bool m_isActive = false;

    float m_elbowAngle = 90;
    float m_elbowAngle2 = 90;
    float m_elbowAngleOld = 90;
    float m_wristAngle = 90;
    float m_wristAngleOld = 90;
    float m_levelAngle = 90;
    float m_levelAngleOld = 90;

    float m_elbowTune = 11;
    float m_elbowTune2 = 12;
    float m_levelTune = 0;
    float m_wristTune = 15;

    float m_levelVelocity = 0.0f;
    float m_wristVelocity = 0.0f;
    
    float const VELOCITY_SCALING_FACTOR = .1f;
    float const DELTA_VELOCITY_IGNORE = .1f;
    float const ELBOW_ANGLE_MAX = 180;
    float const ELBOW_ANGLE_MIN = 0;
    float const LEVEL_ANGLE_MAX = 180;
    float const LEVEL_ANGLE_MIN = 0;
    float const WRIST_ANGLE_MAX = 180;
    float const WRIST_ANGLE_MIN = 0;
};

#endif // MANIPULATOR_HPP
