#include <include/components/CSM_PCA9685.hpp>

PCA9685::PCA9685() {
    // first call i2c_open to open i2c bus
    if((this->bus = i2c_open("/dev/i2c-1")) == -1) {
        // TODO: error
    }

    // set to 0s
    std::memset(buffer, 0, sizeof(buffer));

    // set to 0s
    std::memset(&this->device, 0, sizeof(this->device));
    
    this->device.bus = this->bus;
    this->device.addr =  0x40;
    this->device.iaddr_bytes = 1;
    this->device.page_bytes = 16;
}

void PWMChannel::setDutyCycle(unsigned short dutyIn) {
    
}

unsigned short PWMChannel::getDutyCycle() {
    return this->duty_cycle;
}