#include <include/components/ServoDriver.hpp>

Servo::Servo(ServoDriver driver, int channel){
    this->driver = driver;
    this->channel = channel;
}

void Servo::setLimits(float minimum_limit, float maximum_limit){
    this->minimum_limit = minimum_limit;
    this->maximum_limit = maximum_limit;
}

void Servo::setSetpoint(float setpoint){
    if(setpoint < this->minimum_limit || setpoint > this->maximum_limit) throw "Servo setpoint " + std::to_string(setpoint) + " is not in range [" + std::to_string(minimum_limit) + ", " + std::to_string(maximum_limit) + "]";
    this->setpoint = setpoint;

    // TODO: Convert setpoint to PWM frequency and call this->driver.setPWM(this->channel, frequency)
}
