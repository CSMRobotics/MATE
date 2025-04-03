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
    output_frequency = 200;
}

PCA9685::~PCA9685() {
    close();
}

bool PCA9685::open() {
    char filename[32];
    sprintf(filename,"/dev/i2c-%d", i2c_bus);
    // printf("%s\n", filename);
    i2c_fd = ::open(filename, O_RDWR);
    if (i2c_fd < 0) {
        // Could not open the bus
    
        return false;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0) {
        // Could not open the device on the bus
        return false;
    }

    // TODO: enable and test when external clock is available
    // i2c_smbus_write_byte_data(i2c_fd, MODE1, MODE1_SLEEP);
    // usleep(500);
    // i2c_smbus_write_byte_data(ic2_fd, MODE1, MODE1_SLEEP | MODE1_EXTCLK);

    usleep(500);
    setFrequency(60);

    // clear all registers
    reset();

    return true;
}

void PCA9685::close() {
    setAllOff();
    usleep(500);
    if (i2c_fd > 0) {
        ::close(i2c_fd);
    }
}

void PCA9685::reset() {
    // data sheet restart procedure per 7.3.1.1
    // stop all pwms (orderly shutdown invalidates all PWM registers)
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_OFF_H, BIT4);
    usleep(500);
    
    // if bit 7 is 1, clear bit 4, wait for clock to stabilize
    uint8_t mode1_current = i2c_smbus_read_byte_data(i2c_fd, Registers::MODE1);
    if (mode1_current & MODE1_RESTART) {
        i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, mode1_current & ~MODE1_SLEEP);
        usleep(500);
    }
    // write 1 to bit 7, pwm channels will restart and restart bit will clear
    i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, (mode1_current & ~MODE1_SLEEP) | MODE1_RESTART);
}

void PCA9685::setFrequency(uint16_t frequency) {
    static const uint16_t NUM_COUNTS = 4096; // scaling factor since clock sucks balls lol
    static const uint16_t MIN_FREQUENCY = 24; // Hz
    static const uint16_t MAX_FREQUENCY = 1526; // Hz
    // if given frequency is not valid, do nothing
    if (frequency < MIN_FREQUENCY || MAX_FREQUENCY > 1526) {
        return;
    }
    output_frequency = frequency;
    uint8_t prescale = static_cast<uint8_t>(std::round(clock_frequency/(NUM_COUNTS * frequency))) - 1;
    // printf("prescale: 0x%02x", prescale);
    uint8_t current_mode = i2c_smbus_read_byte_data(i2c_fd, Registers::MODE1);
    // set sleep bit
    i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, current_mode | MODE1_SLEEP);
    usleep(500);
    // set the prescalar
    i2c_smbus_write_byte_data(i2c_fd, Registers::PRE_SCALE, prescale);
    // reset the mode
    i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, current_mode);

    // restart
    usleep(500);
    i2c_smbus_write_byte_data(i2c_fd, Registers::MODE1, current_mode | MODE1_RESTART);
}

void PCA9685::setAllPWM(uint16_t on_counts, uint16_t off_counts) {
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_ON_L, on_counts & 0xFF);
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_ON_H, on_counts >> 8);
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_OFF_L, off_counts & 0xFF);
    i2c_smbus_write_byte_data(i2c_fd, Registers::ALL_LED_OFF_H, off_counts >> 8);
}

void PCA9685::setAllOff() {
    // clear existing full on bit after setting full off (off takes precedence)
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_OFF_H, BIT4);
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_ON_H, i2c_smbus_read_byte_data(i2c_fd, ALL_LED_ON_H) & ~BIT4);
}

void PCA9685::setAllOn() {
    // clear existing full off bit after setting full on (off takes precedence)
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_ON_H, BIT4);
    i2c_smbus_write_byte_data(i2c_fd, ALL_LED_OFF_H, i2c_smbus_read_byte_data(i2c_fd, ALL_LED_OFF_H) & ~BIT4);
}

void PCA9685::setDuty(uint8_t channel, float duty, float _delay) {
    // Hehe duty
    static const uint16_t MAX_COUNT = 4096;
    uint16_t dur_on = static_cast<uint16_t>(std::round(duty * MAX_COUNT));
    uint16_t time_on = static_cast<uint16_t>(std::round(_delay * MAX_COUNT)) - 1;
    if (time_on > MAX_COUNT) {
        time_on = 0;
    }
    uint16_t time_off;
    if(dur_on >= MAX_COUNT){
        // Special case for fully on
        time_on = (1 << 12); // Bit 12 ON means locked-on
        time_off = 0; // The count doesn't matter but A) we're already sending all four on/off bytes and B) if bit 12 is on it will override the ON above
    }else if (dur_on <= 0){
        // Special case for fully off (same arguments as above)
        time_on = 0;
        time_off = (1 << 12); // Bit 12 OFF means locked-off
    }else{
        // General case (clamp delay to 12 bits to cover case of duty/delay time very close to but not quite 1, or when the user sets an invalid delay)
        time_on &= 0xFFF;
        time_off = time_on + dur_on;
        time_off &= 0xFFF; // Modulo Again (in case the off time is in the next frame)
    }

    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_H + (channel << 2), (time_on >> 8) & 0x1F);
    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_L + (channel << 2), time_on & 0xFF);
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_H + (channel << 2), (time_off >> 8) & 0x1F);
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_L + (channel << 2), time_off & 0xFF);
}

void PCA9685::setUS(uint8_t channel, uint16_t us, uint16_t delay_us) {
    // calculate the maximum pulse width at the current output frequency
    float max_pulse_width_us = US_PER_S / output_frequency;
    // calculate the percent duty cycles for on and delay
    float duty = us/max_pulse_width_us;
    float delay = delay_us/max_pulse_width_us;

    printf("%0.4f\n", duty);

    // ensure within [0.0, 1.0]
    duty = duty > 1.0f ? 1.0f : duty;
    duty = duty < 0.0f ? 0.0f : duty;
    delay = delay > 1.0f ? 1.0f : delay;
    delay = delay < 0.0f ? 0.0f : delay;
    // set the duty cycle
    setDuty(channel, duty, delay);
}

void PCA9685::setOn(uint8_t channel) {
    // clear existing full off bit after setting full on
    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_H + (channel << 2), BIT4);
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_H + (channel << 2), i2c_smbus_read_byte_data(i2c_fd, LED0_OFF_H + (channel << 2)) & ~BIT4);
}

void PCA9685::setOff(uint8_t channel) {
    // clear existing full on bit after setting full off
    i2c_smbus_write_byte_data(i2c_fd, LED0_OFF_H + (channel << 2), BIT4);
    i2c_smbus_write_byte_data(i2c_fd, LED0_ON_H + (channel << 2), i2c_smbus_read_byte_data(i2c_fd, LED0_ON_H + (channel << 2)) & ~BIT4);
}
