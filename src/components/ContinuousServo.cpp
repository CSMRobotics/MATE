#include <include/components/ServoDriver.hpp>

ContinuousServo::ContinuousServo(ServoDriver driver, int channel){
    this->driver = driver;
    this->channel = channel;
}

void ContinuousServo::setSetpoint(float setpoint){
    if(setpoint < -1.0f || setpoint > 1.0f) throw "Continuous Servo setpoint " + std::to_string(setpoint) + " is not in range [0, 1]";
    this->setpoint = setpoint;

    // TODO: Convert setpoint to PWM frequency and call this->driver.setPWM(this->channel, frequency)
}