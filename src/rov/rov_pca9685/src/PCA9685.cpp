#include "pca9685/PCA9685.hpp"

extern "C" {
#include "linux/i2c-dev.h"
#include "i2c/smbus.h"
}

#include <cstddef>
#include <sys/ioctl.h>
#include <cstdlib>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include <cmath>

#define US_PER_S 1e6

PCA9685::PCA9685(uint8_t bus, uint8_t address) : i2c_address(address), i2c_bus(bus) {
    clock_frequency = 25e6;
    // 1e6 us per s / 25e6 hz = 0.04 us
    clock_us = US_PER_S / 25e6;
}

PCA9685::~PCA9685() {
    Close();
}

bool PCA9685::Open() {
    char filename[32];
    sprintf(filename,"/dev/i2c-%d", i2c_bus);
    i2c_fd = open(filename, O_RDWR);
    if (i2c_fd < 0) {
        // Could not open the bus
       return false;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
        // Could not open the device on the bus
        return false;
    }

    // external clock?
    // i2c_smbus_write_byte_data(i2c_fd, MODE1, MODE1_SLEEP);
    // usleep(500);
    // i2c_smbus_write_byte_data(ic2_fd, MODE1, MODE1_SLEEP | MODE1_EXTCLK);

    setFrequency(200);

    return true;
}

void PCA9685::Close() {
    setAllOff();
    usleep(500);
    if (i2c_fd > 0) {
        close(i2c_fd);
    }
}

void PCA9685::Reset() {
    // stop all pwms
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_OFF_H, BIT4);
    // if restart is a logic 1
    uint8_t mode1 = i2c_smbus_read_byte_data(i2c_fd, Registers::MODE1);
    if (mode1 & MODE1_RESTART) {
        // clear bit 4
        i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, mode1 & ~MODE1_SLEEP);
        // wait 500 us before setting logic 1 to restart
        usleep(500);
        i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, MODE1_RESTART);
    }
}

void PCA9685::setFrequency(uint16_t frequency) {
    // if given frequency is not valid, do nothing
    if (frequency < 24 || frequency > 1526) {
        return;
    }
    output_frequency = frequency;
    uint8_t prescale = static_cast<uint8_t>(std::round(25e6/(4096 * frequency))) - 1;
    uint8_t current_mode = i2c_smbus_read_byte_data(i2c_fd, Registers::MODE1);
    // set sleep bit
    i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, current_mode | MODE1_SLEEP);
    usleep(500);
    // set the prescalar
    i2c_smbus_write_byte_data(i2c_fd, Registers::PRE_SCALE, prescale);
    // reset the mode
    i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, current_mode);
}

void PCA9685::setAllPWM(uint16_t on_counts, uint16_t off_counts) {
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_ON_L, on_counts & 0xFF);
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_ON_H, on_counts >> 8);
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_OFF_L, off_counts & 0xFF);
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_OFF_H, off_counts >> 8);
}

void PCA9685::setAllOff() {
    // clear existing full on bit before setting full off
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_ON_H, i2c_smbus_read_byte_data(i2c_fd, ALL_LED_ON_H) & ~BIT4);
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_OFF_H, BIT4);
}

void PCA9685::setAllOn() {
    // clear existing full off bit before setting full on
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_OFF_H, i2c_smbus_read_byte_data(i2c_fd, ALL_LED_OFF_H) & ~BIT4);
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_ON_H, BIT4);
}

void PCA9685::setDuty(uint8_t channel, float duty, float _delay) {
    uint16_t time_on = static_cast<uint16_t>(std::round(duty * 4096));
    uint16_t delay = static_cast<uint16_t>(std::round(_delay * 4096));
    uint16_t time_off = delay+time_on - 1;
    time_off = time_off > 4096 ? time_off - 4096 : time_off;
    
    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_H + (channel << 2), time_on >> 8);
    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_L + (channel << 2), time_on & 0xFF);
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_H + (channel << 2), time_off >> 8);
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_L + (channel << 2), time_off & 0xFF);
}

void PCA9685::setUS(uint8_t channel, uint16_t us, uint16_t delay_us) {
    // calculate the maximum pulse width at the current output frequency
    float max_pulse_width_us = US_PER_S / output_frequency;
    // calculate the percent duty cycles for on and delay
    float duty = us/max_pulse_width_us;
    float delay = delay_us/max_pulse_width_us;
    // ensure within [0.0, 1.0]
    duty = duty > 1.0f ? 1.0f : duty;
    duty = duty < 0.0f ? 0.0f : duty;
    delay = delay > 1.0f ? 1.0f : delay;
    delay = delay < 0.0f ? 0.0f : delay;
    // set the duty cycle
    setDuty(channel, duty, delay);
}

void PCA9685::setOn(uint8_t channel) {
    // clear existing full off bit before setting full on
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_H + (channel << 2), i2c_smbus_read_byte_data(i2c_fd, LED0_OFF_H + (channel << 2)) & ~BIT4);
    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_H + (channel << 2), BIT4);
}

void PCA9685::setOff(uint8_t channel) {
    // clear existing full on bit before setting full off
    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_H + (channel << 2), i2c_smbus_read_byte_data(i2c_fd, LED0_ON_H + (channel << 2)) & ~BIT4);
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_H + (channel << 2), BIT4);
}