#include "BAR02.hpp"

#include <math.h>
#include <limits.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

csmutil::MS5837::MS5837(int address) {
    // create fd to i2c file
    const char* filename = "/dev/i2c-1";
    m_fd = open(filename, O_RDWR);
    if(m_fd < 0) {
        perror("Failed to open I2c bus file");
        exit(1);
    }
    if(ioctl(m_fd, I2C_SLAVE, address) < 0) {
        perror("Failed to open i2c bus");
        exit(1);
    }
    
    // initialize any member variables that need to be
}

void csmutil::MS5837::Update() {

}

void csmutil::MS5837::AutoUpdate() {
    Update();
}

void csmutil::MS5837::Stop() {
    return;
}

float csmutil::MS5837::pressure(float conversion) {
    
}

float csmutil::MS5837::temperature() {

}

float csmutil::MS5837::depth() {

}

float csmutil::MS5837::altitude() {

}

void csmutil::MS5837::calculate() {

}

uint8_t csmutil::MS5837::readByte(uint8_t address) {
    return i2c_smbus_read_byte_data(m_fd, address);
}

signed int csmutil::MS5837::writeByte(uint8_t address, uint8_t byte) {
    return i2c_smbus_write_byte_data(m_fd, address, byte);
}