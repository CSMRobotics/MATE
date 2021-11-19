#include "Drive.hpp"

Drive::Drive(ServoDriver* driver, Joystick* joystick, IMU* imu) {
    this->driver = driver;
    this->joystick = joystick;
    this->imu = imu;
    
    pid = csmutil::NonLinearQuaternionController(0, 0);

    uint64_t timeInitialMillis = millis();
    timeCurrentMillis = timeInitialMillis;
    timePreviousMillis = timeInitialMillis;

    dt = 0;
}

Drive::~Drive() {

}

void Drive::Update() {
    // update modes
    ButtonPresses presses = joystick->getPresses();
    if(activeComponent && presses[1]) { // if mode switch button (button 1) is pressed, switch modes
        modeState = ModeState(!modeState);
    }

    // TODO: implement state controller for components;

    if (presses[2]) { // if active component button is pressed, make this active component
        if(activeComponent) {// if already active, switch state
            state = DriveState((state+1) %3); // switch between modes 0/1/2
        }
    }

    


    // always get update from PID controller (even if you arent going to use it!)
    timePreviousMillis = timeCurrentMillis;
    timeCurrentMillis = millis();
    dt = timeCurrentMillis - timePreviousMillis;

    axes = joystick->getAxes();
    if(state == PID_ATTITUDE)
        q_ref = createRef(axes);

    torque = pid.Update(q_ref, imu->getQuaternion(), imu->getGyro());

    switch(state) {
        case MANUAL:

            break;
        case PID_ATTITUDE:
            // TODO: map torque to throttles
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