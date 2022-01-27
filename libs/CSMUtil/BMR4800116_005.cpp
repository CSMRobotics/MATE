#include "BMR4800116_005.hpp"


BMR4800116_005::BMR4800116_005() {
    const char* filename = "/dev/i2c-1";
    m_fd = open(filename, O_RDWR);
    if(m_fd < 0) {
        perror("Failed to open I2C bus file");
        exit(1);
    }
    if(ioctl(m_fd, I2C_SLAVE, BMR4800116_005_ADDRESS) < 0) {
        perror("Failed to open I2C bus");
        exit(1);
    }    
}

bool BMR4800116_005::begin(uint8_t operationMode) {
    usleep(5000);
    // setup device in valid state
    // Enabled, Nominal output voltage, ignore faults
    writeByte(BMR4800116_005_OPERATION, operationMode);
}

bool BMR4800116_005::getPowerStats(PowerStats* ps) {
    ps->inputVoltage = readWord(BMR4800116_005_READ_VIN);
    ps->outputVoltage = readWord(BMR4800116_005_READ_VOUT);
    ps->outputCurrent = readWord(BMR4800116_005_READ_IOUT);
    ps->temp1 = readWord(BMR4800116_005_READ_TEMPERATURE_1);
    ps->temp2 = readWord(BMR4800116_005_READ_TEMPERATURE_2);
    return true;
}


uint16_t BMR4800116_005::readWord(uint8_t reg) {
    return i2c_smbus_read_word_data(m_fd, reg);
}

uint8_t BMR4800116_005::readByte(uint8_t reg) {
    return i2c_smbus_read_byte_data(m_fd, reg);
}

bool BMR4800116_005::writeByte(uint8_t reg, uint8_t value) {
    return (!i2c_smbus_write_byte_data(m_fd, reg, value) < 0);
}

bool BMR4800116_005::writeWord(uint8_t reg, uint16_t value) {
    return (!i2c_smbus_write_word_data(m_fd, reg, value) < 0);
}