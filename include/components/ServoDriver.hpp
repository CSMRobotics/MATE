#include <libs/JHPWMPCA9685/JHPWMPCA9685.h>

class ServoDriver{
    public:
        ServoDriver();
        ~ServoDriver();

        Servo* addServo(int channel);
        ContinuousServo* addContinuousServo(int channel);
        ServoBase* get(int channel);
        void setPWM(int channel, int value);
        void setPWMFrequency(float frequency);

    private:
        static PCA9685 driver_board = PCA9685();
        ServoBase* servos[16] = {0};
}