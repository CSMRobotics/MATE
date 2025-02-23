#ifndef SERVO_DRIVER_HPP
#define SERVO_DRIVER_HPP

#include "pca9685/PCA9685.hpp"
#include <string>

enum class ServoType{
    POSITIONAL,
    CONTINUOUS
};

struct Servo{
    bool initialized = false;
    ServoType type;
    float setpoint = 0.0f;
    float setpoint_minimum = 0.0f;
    float setpoint_maximum = 0.0f;
    int pwm_minimum;
    int pwm_maximum;
};

class ServoDriver{
    public:
        ServoDriver();
        ~ServoDriver();

        void addServo(int channel);
        void addContinuousServo(int channel);
        void removeServo(int channel);
        void removeContinuousServo(int channel);
        void setAngle(int channel, float angle);
        void setThrottle(int channel, float throttle);
        void setOutput(int channel, float value);
        void setAngleBounds(int channel, float minimum_angle, float maximum_angle);
        void setThrottleBounds(int channel, float minimum_throttle, float maximum_throttle);
        void setPWMBounds(int channel, int minimum_us, int maximum_us);

    private:
        PCA9685 driver_board;
        float driver_board_frequency = 60;
        Servo servos[16];

        void setPWM(int channel, int value);
        void setPWMFrequency(float frequency);
        bool isChannelInUse(int channel);
        bool isChannelNotInUse(int channel);
        bool isServoChannelInUse(int channel);
        bool isContinuousServoChannelInUse(int channel);
        float getCountsPerMicrosecond();
};

#endif // SERVO_DRIVER_HPP