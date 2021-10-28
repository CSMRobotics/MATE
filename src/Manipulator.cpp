#include "Manipulator.hpp"

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
    // update velocities based on pitch and roll joystick axes
    m_levelVelocity = joystick->getAxes()[1];
    m_wristVelocity = joystick->getAxes()[0];

    if(joystick->getPresses()[m_clampButton])
        m_isClamping = !m_isClamping;
    if(joystick->getPresses()[m_chickenButton])
        m_isChicken = !m_isChicken;

    if(m_isChicken){
        m_elbowAngle = m_elbowAngleOld + m_levelVelocity;
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

    if(m_levelAngle > LEVEL_ANGLE_MAX)
        m_levelAngle = LEVEL_ANGLE_MAX;
    else if(m_levelAngle < LEVEL_ANGLE_MIN)
        m_levelAngle = LEVEL_ANGLE_MIN;

    if(m_wristAngle > WRIST_ANGLE_MAX)
        m_wristAngle = WRIST_ANGLE_MAX;
    else if(m_wristAngle < WRIST_ANGLE_MIN)
        m_wristAngle = WRIST_ANGLE_MIN;

    // update positions
    // always write wrist
    this->driver->setAngle(m_wristServo, m_wristAngle + m_wristTune);

    //determines updates based on chicken protocol
    if(m_isChicken) {
        this->driver->setAngle(m_elbowServo, m_elbowAngle + m_elbowTune);
        this->driver->setAngle(m_elbowServo2, 180 - m_elbowAngle + m_elbowTune2);
        this->driver->setAngle(m_levelServo, m_levelAngle + m_levelTune);
    }
    else {
        this->driver->setAngle(m_levelServo, m_levelAngle + m_levelTune);
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