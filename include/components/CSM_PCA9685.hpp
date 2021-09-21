#include <libs/libi2c/include/i2c/i2c.h>
#include <cstring>

class PWMChannel {
public:
    void setDutyCycle(unsigned short dutyIn);
    unsigned short getDutyCycle();

private:
    int index;
    PCA9685* pca;
    unsigned short duty_cycle;
};

class PCAChannels {
    // lazily creates channel objects as needed
public:
    PWMChannel get(int i);
protected:
    PCAChannels();
    ~PCAChannels();
private:
    PWMChannel* channels[16] = {nullptr};
    PCA9685* pca;
};

class PCA9685 {
public:
    PCA9685(); // Initialise PCA9685 at address on i2c_bus
    ~PCA9685();

private:
    int bus;
    I2CDevice device;
    unsigned char buffer[256];
    ssize_t size = sizeof(buffer);
    PCAChannels* channels;
    // TODO: create data structure to hold the PWM Regulation data (high and low for each channel)
};