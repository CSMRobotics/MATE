#ifndef BNO08X_DRIVER_HPP
#define BNO08X_DRIVER_HPP

#include <cstdint>

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#define I2C_DEFAULT_ADDRESS 0x4A

#define PAC_ON_STAIRS 8
#define PAC_OPTION_COUNT 9

// NOTE: how to write to gpio? -> https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
// EXAMPLE: https://developer.ridgerun.com/wiki/index.php/Gpio-int-test.c
// NOTE: EXECUTABLE NEEDS ACCESS TO SYSFS (root only) https://unix.stackexchange.com/a/409780
// MAYBE SET UDEV? https://forums.developer.nvidia.com/t/gpio-permissions-problem-udev-rules-not-working/267078/5
//      https://github.com/NVIDIA/jetson-gpio/blob/master/lib/python/Jetson/GPIO/99-gpio.rules
#include "rov_bno085/setup_gpio_exports.hpp"

// ABOVE IS DEPRECATED??
// https://forums.developer.nvidia.com/t/orin-nano-fast-gpio-c-library-with-direct-register-access/303681
// https://docs.kernel.org/userspace-api/gpio/chardev.html
// looks like this works? https://github.com/Rubberazer/JETGPIO?tab=readme-ov-file
//                      https://github.com/Rubberazer/Jetclocks

// see https://github.com/adafruit/Adafruit_BNO08x/blob/master/src/Adafruit_BNO08x.cpp
// for inspiration
class BNO08X {
public:
    BNO08X(uint8_t reset_pin);

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

    int interrupt_pin;

protected:
    // called post i2c/spi init
    virtual bool _init(int32_t sensor_id);

    sh2_Hal_t _HAL;
};

#endif