#ifndef SERVO_DRIVER_HPP
#define SERVO_DRIVER_HPP

#include "pca9685/PCA9685.hpp"
#include <string>
#include <map>

enum class ServoType : uint8_t {
    POSITIONAL,
    CONTINUOUS
};

struct ServoBase {
    bool initialized = false;
};

struct Servo : ServoBase {
    uint16_t us_minimum = 1100;
    uint16_t us_maximum = 1900;
};

struct ContinuousServo : ServoBase {
    uint16_t us_minimum = 1100;
    uint16_t us_maximum = 1900;
};

class ServoDriver {
public:
    ServoDriver();
    ~ServoDriver();

    void registerServo(uint8_t channel, ServoType type);

    void setDuty(uint8_t channel, float duty);
    void setUS(uint8_t channel, uint16_t us);

    void setThrottle(uint8_t channel, float throttle);
    void setAngle(uint8_t channel, float angle);

    void setUSBounds(uint8_t channel, uint16_t min_us, uint16_t max_us);

    void setOutput(uint8_t channel, float angle_or_throttle);
private:
    PCA9685 driver_board;
    std::map<uint8_t, Servo> servos;
    std::map<uint8_t, ContinuousServo> continuous_servos;
};

#endif
