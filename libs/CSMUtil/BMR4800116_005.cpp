#include "BMR4800116_005.hpp"


BMR4800116_005::BMR4800116_005(uint8_t operationMode=0x84) {
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

    usleep(50000); // wait for stable connection

    // setup device in valid state
    // Enabled, Nominal output voltage, ignore faults
    writeByte(BMR4800116_005_OPERATION, operationMode);
}

PowerStats BMR4800116_005::getPowerStats() {
    PowerStats toRet;
    toRet.inputVoltage = readWord(BMR4800116_005_READ_VIN);
    toRet.outputVoltage = readWord(BMR4800116_005_READ_VOUT);
    toRet.outputCurrent = readWord(BMR4800116_005_READ_IOUT);
    toRet.temp1 = readWord(BMR4800116_005_READ_TEMPERATURE_1);
    toRet.temp2 = readWord(BMR4800116_005_READ_TEMPERATURE_2);
    return toRet;
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