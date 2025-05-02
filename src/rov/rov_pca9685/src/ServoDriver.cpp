#include "pca9685/ServoDriver.hpp"
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#define NUM_COUNTS 4177
#define BUS_DEV 1
#define ADDRESS 0x58

static int f2imap(float value, float from_min, float from_max, int to_min, int to_max){
    int val = ((value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min);
    val = val < to_min ? to_min : val;
    val = val > to_max ? to_max : val;
    return val;
}

ServoDriver::ServoDriver() {
    this->driver_board = PCA9685(BUS_DEV, ADDRESS);
    if(!this->driver_board.open()) {
        throw std::runtime_error("Failed to open PCA9685");
    }
    this->driver_board.setAllOff();
}

ServoDriver::~ServoDriver() {}

void ServoDriver::registerServo(uint8_t channel, ServoType type) {
    switch (type)
    {
    case ServoType::POSITIONAL:
        servos[channel] = Servo();
        RCLCPP_INFO(rclcpp::get_logger("ServoDriver"), "Registered positional servo on channel %d", channel);
        setAngle(channel, 90.0f);
        break;
    
    case ServoType::CONTINUOUS:
        continuous_servos[channel] = ContinuousServo();
        RCLCPP_INFO(rclcpp::get_logger("ServoDriver"), "Registered continuous servo on channel %d", channel);
        setThrottle(channel, 0.0f);
        break;
    }
}

void ServoDriver::setDuty(uint8_t channel, float duty) {
    driver_board.setDuty(channel, duty);
}

void ServoDriver::setUS(uint8_t channel, uint16_t us) {
    driver_board.setUS(channel, us);
}

void ServoDriver::setThrottle(uint8_t channel, float throttle) {
    if (!continuous_servos.count(channel)) {
        RCLCPP_WARN(rclcpp::get_logger("ServoDriver"), "Attempted to set throttle on non-continuous servo");
        return;
    }
    ContinuousServo* s = &continuous_servos[channel];
    driver_board.setUS(channel, f2imap(throttle, -1.0, 1.0, s->us_minimum, s->us_maximum));
}

void ServoDriver::setAngle(uint8_t channel, float angle) {
    if (!servos.count(channel)) {
        RCLCPP_WARN(rclcpp::get_logger("ServoDriver"), "Attempted to set angle on continuous servo");
        return;
    }
    Servo* s = &servos[channel];
    driver_board.setUS(channel, f2imap(angle, 0.0, 180.0, s->us_minimum, s->us_maximum));
}

void ServoDriver::setUSBounds(uint8_t channel, uint16_t min_us, uint16_t max_us) {
    if (servos.count(channel)) {
        servos[channel].us_minimum = min_us;
        servos[channel].us_maximum = max_us;
        return;
    } else if (continuous_servos.count(channel)) {
        continuous_servos[channel].us_minimum = min_us;
        continuous_servos[channel].us_maximum = max_us;
        return;
    }
    RCLCPP_WARN(rclcpp::get_logger("ServoDriver"), "Attempted to set the bounds of an unregistered servo");
}

void ServoDriver::setOutput(uint8_t channel, float angle_or_throttle) {
    if (servos.count(channel)) {
        setAngle(channel, angle_or_throttle);
        return;
    } else if (continuous_servos.count(channel)) {
        setThrottle(channel, angle_or_throttle);
        return;
    }
    RCLCPP_WARN(rclcpp::get_logger("ServoDriver"), "Attempted to set the output of an unregistered servo");
}