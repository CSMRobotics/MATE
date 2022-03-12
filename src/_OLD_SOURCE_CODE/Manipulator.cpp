#include "Manipulator.hpp"
#include <iostream>

Manipulator::Manipulator(Joystick* joystick, ServoDriver* driver) {
    this->joystick = joystick;
    this->driver = driver;
    m_isActive = false;
    m_isChicken = false;
    m_isClamping = false;
    this->driver->addServo(m_elbowServo);
    this->driver->addServo(m_elbowServo2);
    this->driver->addServo(m_levelServo);
    this->driver->addServo(m_wristServo);
    this->driver->addServo(m_clampServo);
}

void Manipulator::Update() {
    // check if this component should listen to joystick
    if(!m_isActive) {
        return;
    }

    // get button presses
    ButtonPresses presses = joystick->getPresses();

    // update velocities based on pitch and roll joystick axes
    m_levelVelocity = joystick->getAxes()[1];
    m_wristVelocity = joystick->getAxes()[0];
    // std::cout << m_wristVelocity << std::endl;

    if(presses[m_clampButton]) {
        m_isClamping = !m_isClamping;
	std::cout << "clamp toggled to " << m_isClamping << std::endl;
    }
    if(presses[m_chickenButton]) {
        m_isChicken = !m_isChicken;
	std::cout << "chicken toggled to " << m_isChicken << std::endl;
    }

    if(m_isChicken) {
        m_elbowAngle = m_elbowAngleOld + m_levelVelocity;
	    m_elbowAngle2 = 180 - m_elbowAngle;
        m_levelAngle = m_levelAngleOld - m_levelVelocity;
        m_wristAngle = m_wristAngleOld + m_wristVelocity;
    }
    else {
        m_levelAngle = m_levelAngleOld + m_levelVelocity;
        m_wristAngle = m_wristAngleOld + m_wristVelocity;
    }

    // keep angles from over/under-shooting
    if(m_elbowAngle > ELBOW_ANGLE_MAX)
        m_elbowAngle = ELBOW_ANGLE_MAX;
    else if(m_elbowAngle < ELBOW_ANGLE_MIN)
        m_elbowAngle = ELBOW_ANGLE_MIN;

    if(m_elbowAngle2 > ELBOW_ANGLE_MAX)
	    m_elbowAngle2 = ELBOW_ANGLE_MAX;
    else if(m_elbowAngle2 < ELBOW_ANGLE_MIN)
        m_elbowAngle2 = ELBOW_ANGLE_MIN;

    if(m_levelAngle > LEVEL_ANGLE_MAX)
        m_levelAngle = LEVEL_ANGLE_MAX;
    else if(m_levelAngle < LEVEL_ANGLE_MIN)
        m_levelAngle = LEVEL_ANGLE_MIN;

    if(m_wristAngle > WRIST_ANGLE_MAX)
        m_wristAngle = WRIST_ANGLE_MAX;
    else if(m_wristAngle < WRIST_ANGLE_MIN)
        m_wristAngle = WRIST_ANGLE_MIN;
    
    // std::cout << "Elbow Angle:  " << m_elbowAngle << '\n';
    // std::cout << "Elbow Angle2: " << m_elbowAngle2 << '\n';
    // std::cout << "Level Angle:  " << m_levelAngle << '\n';
    // std::cout << "Wrist Angle:  " << m_wristAngle << std::endl;

    // update positions
    // always write wrist
    this->driver->setAngle(m_wristServo, m_wristAngle);

    //determines updates based on chicken protocol
    if(m_isChicken) {
        this->driver->setAngle(m_elbowServo, m_elbowAngle);
        this->driver->setAngle(m_elbowServo2, m_elbowAngle2);
        this->driver->setAngle(m_levelServo, m_levelAngle);
    }
    else {
        this->driver->setAngle(m_levelServo, m_levelAngle);
    }

    // update vars
    m_elbowAngleOld = m_elbowAngle;
    m_levelAngleOld = m_levelAngle;
    m_wristAngleOld = m_wristAngle;

    // TODO: reimplement this to prevent unnecessary torque from being applied to gripped object (overheating possible issue???)
    if(m_isClamping) {
        this->driver->setAngle(m_clampServo, 130);
    }
    else {
        this->driver->setAngle(m_clampServo, 85);
    }
}

void Manipulator::AutoUpdate() {
    this->Update();
}

void Manipulator::Stop() {
    this->driver->setAngle(m_elbowServo, 0);
    this->driver->setAngle(m_elbowServo2, 0);
    this->driver->setAngle(m_levelServo, 0);
    this->driver->setAngle(m_wristServo, 0);
    this->driver->setAngle(m_clampServo, 0);
}
