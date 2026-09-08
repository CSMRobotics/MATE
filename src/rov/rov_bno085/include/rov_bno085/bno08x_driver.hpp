#ifndef BNO08X_DRIVER_HPP
#define BNO08X_DRIVER_HPP

#include <cstdint>

extern "C" {
#include "sh2_SensorValue.h"
#include "sh2_hal.h"
}

#define I2C_DEFAULT_ADDRESS 0x4A
#define DEFAULT_RESET_PIN 33
#define DEFAULT_INTERRUPT_PIN 31

#define PAC_ON_STAIRS 8
#define PAC_OPTION_COUNT 9

// looks like this works? https://github.com/Rubberazer/JETGPIO?tab=readme-ov-file
//                      https://github.com/Rubberazer/Jetclocks
// ^^^^ TODO: NEED TO INSTALL JETCLOCKS ON ORIN

// see https://github.com/adafruit/Adafruit_BNO08x/blob/master/src/Adafruit_BNO08x.cpp
// for inspiration
class BNO08X {
public:
    BNO08X(uint8_t reset_pin = DEFAULT_RESET_PIN, uint8_t interrupt_pin = DEFAULT_INTERRUPT_PIN);
    BNO08X(BNO08X&&) = delete;
    BNO08X operator=(BNO08X&) = delete;
    ~BNO08X();

    bool makeI2C(uint8_t address = I2C_DEFAULT_ADDRESS, int32_t sensor_id = 0) {return false;}; // NOTE: UNIMPLEMENTED
    bool makeUART(int32_t sensor_id = 0) {return false;}; // NOTE: UNIMPLEMENTED
    bool makeSPI(int32_t sensor_id = 0);

    // reset the device using the reset pin
    void hardwareReset(void);
    // check if a reset has occured
    bool wasReset(void);

    // enable the given report type
    bool enableReport(sh2_SensorId_t sensor_id, uint32_t interval_us = 10000);
    // fill the given sensor value object with a new report
    bool getSensorEvent(sh2_SensorValue_t* value);
private:
    sh2_ProductIds_t product_ids;

    int fd;

    uint8_t reset_pin;
    uint8_t interrupt_pin;

protected:
    // called post i2c/spi init
    virtual bool _init(int32_t sensor_id);

    sh2_Hal_t _HAL;
};

#endif