#include "JHPWMPCA9685.h"
#include <string>

struct Servo{
    bool initialized = false;
    std::string type = "";
    float setpoint = 0.0f;
    float setpoint_minimum = 0.0f;
    float setpoint_maximum = 0.0f;
    int pwm_minimum = 500;
    int pwm_maximum = 2500;
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
        void setAngleBounds(int channel, float minimum_angle, float maximum_angle);
        void setThrottleBounds(int channel, float minimum_throttle, float maximum_throttle);
        void setPWMBounds(int channel, float minimum_pwm, float maximum_pwm);

    private:
        PCA9685 driver_board;
        Servo servos[16];

        void setPWM(int channel, int value);
        void setPWMFrequency(float frequency);
        bool isChannelInUse(int channel);
        bool isChannelNotInUse(int channel);
        bool isServoChannelInUse(int channel);
        bool isContinuousServoChannelInUse(int channel);
};