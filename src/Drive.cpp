#include "Drive.hpp"

Drive::Drive(ServoDriver* driver, Joystick* joystick, IMU* imu) {
    this->m_driver = driver;
    this->m_joystick = joystick;
    this->m_imu = imu;
    this->m_orientation = csmutil::Quaterniond();
    
    m_attitudePID = csmutil::NonLinearQuaternionController(0, 0);

    uint64_t timeInitialMillis = millis();
    m_timeCurrentMillis = timeInitialMillis;
    m_timePreviousMillis = timeInitialMillis;

    m_dt = 0;
}

Drive::~Drive() {

}

csmutil::Quaternionf Drive::createRef(Axes axes) {
    return csmutil::Quaternionf(0,0,0,0);
}

void Drive::Update() {
    // grab all data
    ButtonPresses presses = m_joystick->getPresses();
    Axes axes = m_joystick->getAxes();
    m_linaccel = m_imu->m_NDOF_Data.linearaccel;
    m_orientationLast = m_orientation;
    m_orientation = m_imu->m_NDOF_Data.orientation;
    // m_angaccel = (m_orientationLast - m_orientation)

    // update modes
    if(m_isActive && presses[1]) { // if mode switch button (button 1) is pressed, switch modes
        modeState = ModeState(!modeState);
    }

    // TODO: implement state controller for components;

    if (presses[2]) { // if active component button is pressed, make this active component
        if(m_isActive) {// if already active, switch state
            state = DriveState((state+1) %3); // switch between modes 0/1/2
        }
    }

    


    // always get update from PID controller (even if you arent going to use it!)
    m_timePreviousMillis = m_timeCurrentMillis;
    m_timeCurrentMillis = millis();
    m_dt = m_timeCurrentMillis - m_timePreviousMillis;

    if(state == PID_ATTITUDE) {
        // m_q_ref = createRef(m_axes);
    }

    // m_desiredTorque = pid.Update(q_ref, imu->getQuaternion(), imu->getGyro());

    switch(state) {
        case MANUAL:

            break;
        case PID_ATTITUDE:
            
            break;
        case AUTO:
            break;
    }
}

void Drive::AutoUpdate() {

}

void Drive::Stop() {

}

float Thruster::calculateThrustAtPoint(float throttle) {
    if(throttle < -.1) {
        return -1.7133 * throttle * throttle + 1.4147 * throttle + .1166;
    }
    else if(throttle > .1) {
        return 2.4039 * throttle * throttle + 1.5728 * throttle - .1139;
    }
    
    return 0;
}

void Thruster::set(float throttle) {
    this->throttle = throttle;
    driver->setThrottle(num, throttle);
}