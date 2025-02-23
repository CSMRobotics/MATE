#include "pca9685/ServoDriver.hpp"
#include <iostream>

#define NUM_COUNTS 4177
#define BUS_DEV 7
#define ADDRESS 0x40

static int f2imap(float value, float from_min, float from_max, int to_min, int to_max){
    return ((value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min);
}

ServoDriver::ServoDriver(){
    this->driver_board = PCA9685(BUS_DEV, ADDRESS);
    if(!this->driver_board.Open()) {
        throw std::runtime_error("Failed to open PCA9685");
    }
    this->driver_board.setAllPWM(0,0);
    this->driver_board.Reset();
    this->setPWMFrequency(this->driver_board_frequency);
}

ServoDriver::~ServoDriver(){}

void ServoDriver::addServo(int channel){
    if(this->isChannelNotInUse(channel)){
        Servo new_servo;
        new_servo.initialized = true;
        new_servo.type = ServoType::POSITIONAL;
        new_servo.setpoint_minimum = 0.0f;
        new_servo.setpoint_maximum = 180.0f;
        this->servos[channel] = new_servo;
        this->setPWMBounds(channel, 500, 2500);
    }
}

void ServoDriver::addContinuousServo(int channel){
    if(this->isChannelNotInUse(channel)){
        Servo new_servo;
        new_servo.initialized = true;
        new_servo.type = ServoType::CONTINUOUS;
        new_servo.setpoint_minimum = -1.0f;
        new_servo.setpoint_maximum = 1.0f;
        this->servos[channel] = new_servo;
        this->setPWMBounds(channel, 1100, 1900);
    }
}

void ServoDriver::removeServo(int channel){
    if(this->isServoChannelInUse(channel)){
        this->servos[channel].initialized = false;
    }
}

void ServoDriver::removeContinuousServo(int channel){
    if(this->isContinuousServoChannelInUse(channel)){
        this->servos[channel].initialized = false;
    }
}

void ServoDriver::setAngle(int channel, float angle){
    if(this->isServoChannelInUse(channel)){
        Servo* working_servo = &this->servos[channel];
        if(!(angle >= working_servo->setpoint_minimum && angle <= working_servo->setpoint_maximum)) {
            throw std::domain_error("Servo angle" + std::to_string(angle) + " is not in range [" + std::to_string(working_servo->setpoint_minimum) + ", " + std::to_string(working_servo->setpoint_maximum) + "]");
        }
        working_servo->setpoint = angle;
        this->setPWM(channel, f2imap(working_servo->setpoint, working_servo->setpoint_minimum, working_servo->setpoint_maximum, working_servo->pwm_minimum, working_servo->pwm_maximum));
    } else {
        std::cerr << "[ERROR] Attempted to set an angle for a continuous servo" << std::endl;
    }
}

void ServoDriver::setThrottle(int channel, float throttle){
    if(this->isContinuousServoChannelInUse(channel)){
        Servo* working_servo = &this->servos[channel];
	    if(!(throttle >= working_servo->setpoint_minimum && throttle <= working_servo->setpoint_maximum)) {
            throw std::domain_error("Servo angle " + std::to_string(throttle) + "is not in range [" + std::to_string(working_servo->setpoint_minimum) + ", " + std::to_string(working_servo->setpoint_maximum) + "]");
        }
        working_servo->setpoint = throttle;
        this->setPWM(channel, f2imap(working_servo->setpoint, working_servo->setpoint_minimum, working_servo->setpoint_maximum, working_servo->pwm_minimum, working_servo->pwm_maximum));
	
    } else {
        std::cerr << "[ERROR] Attempted to set a throttle for a positional servo" << std::endl;
    }
}

void ServoDriver::setOutput(int channel, float value) {
    if(isChannelInUse(channel)) {
        switch(servos[channel].type) {
            case ServoType::CONTINUOUS:
                setThrottle(channel, value);
            break;
            case ServoType::POSITIONAL:
                setAngle(channel, value);
            break;
        }
    }
}

void ServoDriver::setAngleBounds(int channel, float minimum_angle, float maximum_angle){
    if(this->isServoChannelInUse(channel)){
        Servo* working_servo = &this->servos[channel];
        working_servo->setpoint_minimum = minimum_angle;
        working_servo->setpoint_maximum = maximum_angle;
    }
}

void ServoDriver::setThrottleBounds(int channel, float minimum_throttle, float maximum_throttle){
    if(this->isContinuousServoChannelInUse(channel)){
        Servo* working_servo = &this->servos[channel];
        working_servo->setpoint_minimum = minimum_throttle;
        working_servo->setpoint_maximum = maximum_throttle;
    }
}

void ServoDriver::setPWMBounds(int channel, int minimum_us, int maximum_us){
    if(this->isChannelInUse(channel)){
        Servo* working_servo = &this->servos[channel];
        float counts_per_microsecond = this->getCountsPerMicrosecond();
	    working_servo->pwm_minimum = minimum_us * counts_per_microsecond;
        working_servo->pwm_maximum = maximum_us * counts_per_microsecond;
    }
}

void ServoDriver::setPWM(int channel, int value){
    if(this->isChannelInUse(channel)){
        this->driver_board.setPWM(channel, 0, value);
    }
}

void ServoDriver::setPWMFrequency(float frequency){
    this->driver_board.setPWMFrequency(frequency);
}

bool ServoDriver::isChannelInUse(int channel){
    if(!(channel >= 0 && channel <= 15)) {
        throw std::domain_error("Channel number " + std::to_string(channel) + " is not in range [0, 15]");
    }
    return this->servos[channel].initialized;
}

bool ServoDriver::isChannelNotInUse(int channel){
    return !isChannelInUse(channel);
}

bool ServoDriver::isServoChannelInUse(int channel){
    if(this->isChannelInUse(channel)) {
        if(servos[channel].type != ServoType::POSITIONAL) {
            throw std::logic_error("Requested servo is already initialized as continuous");
        }
        return true;
    }else{
        return false;
    }
}

bool ServoDriver::isContinuousServoChannelInUse(int channel){
    if(this->isChannelInUse(channel)) {
        if(servos[channel].type != ServoType::CONTINUOUS) {
            throw std::logic_error("Requested servo is already initialized as positional");
        }
        return true;
    } else {
        return false;
    }
}

float ServoDriver::getCountsPerMicrosecond(){
    return (this->driver_board_frequency * NUM_COUNTS) / 1e6;
}
