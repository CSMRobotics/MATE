#include "Drive.hpp"

Drive::Drive(Joystick* joystick, IMU* imu, Thruster* thrusters[]) {
    this->joystick = joystick;
    this->imu = imu;
    for (int i = 0; i < sizeof(thrusters); i++) {
        this->thrusters[i] = thrusters[i];
    }
    timeInitialMillis = millis();
    timeCurrentMillis = timeInitialMillis;
}

Drive::~Drive() {

}

void Drive::Update() {
    timePreviousMillis = timeCurrentMillis;
    timeCurrentMillis = millis();
}

void Drive::AutoUpdate() {

}

void Drive::Stop() {

}