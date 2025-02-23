#ifndef PCA9685_HPP
#define PCA9685_HPP

#include <cstdint>

namespace {
    const uint8_t BIT7 = 0x80;
    const uint8_t BIT6 = 0x40;
    const uint8_t BIT5 = 0x20;
    const uint8_t BIT4 = 0x10;
    const uint8_t BIT3 = 0x08;
    const uint8_t BIT2 = 0x04;
    const uint8_t BIT1 = 0x02;
    const uint8_t BIT0 = 0x01;
}

class PCA9685 {
public:
    enum Registers : uint8_t {
        MODE1           = 0x00,
        MODE2           = 0x01,
        SUBADR1         = 0x02,
        SUBADR2         = 0x03,
        SUBADR3         = 0x04,
        ALLCALLADR      = 0x05,
        LED0_ON_L       = 0x06,
        LED0_ON_H       = 0x07,
        LED0_OFF_L      = 0x08,
        LED0_OFF_H      = 0x09,
        LED1_ON_L       = 0x0a,
        LED1_ON_H       = 0x0b,
        LED1_OFF_L      = 0x0c,
        LED1_OFF_H      = 0x0d,
        LED2_ON_L       = 0x0e,
        LED2_ON_H       = 0x0f,
        LED2_OFF_L      = 0x10,
        LED2_OFF_H      = 0x11,
        LED3_ON_L       = 0x12,
        LED3_ON_H       = 0x13,
        LED3_OFF_L      = 0x14,
        LED3_OFF_H      = 0x15,
        LED4_ON_L       = 0x16,
        LED4_ON_H       = 0x17,
        LED4_OFF_L      = 0x18,
        LED4_OFF_H      = 0x19,
        LED5_ON_L       = 0x1a,
        LED5_ON_H       = 0x1b,
        LED5_OFF_L      = 0x1c,
        LED5_OFF_H      = 0x1d,
        LED6_ON_L       = 0x1e,
        LED6_ON_H       = 0x1f,
        LED6_OFF_L      = 0x20,
        LED6_OFF_H      = 0x21,
        LED7_ON_L       = 0x22,
        LED7_ON_H       = 0x23,
        LED7_OFF_L      = 0x24,
        LED7_OFF_H      = 0x25,
        LED8_ON_L       = 0x26,
        LED8_ON_H       = 0x27,
        LED8_OFF_L      = 0x28,
        LED8_OFF_H      = 0x29,
        LED9_ON_L       = 0x2a,
        LED9_ON_H       = 0x2b,
        LED9_OFF_L      = 0x2c,
        LED9_OFF_H      = 0x2d,
        LED10_ON_L      = 0x2e,
        LED10_ON_H      = 0x2f,
        LED10_OFF_L     = 0x30,
        LED10_OFF_H     = 0x31,
        LED11_ON_L      = 0x32,
        LED11_ON_H      = 0x33,
        LED11_OFF_L     = 0x34,
        LED11_OFF_H     = 0x35,
        LED12_ON_L      = 0x36,
        LED12_ON_H      = 0x37,
        LED12_OFF_L     = 0x38,
        LED12_OFF_H     = 0x39,
        LED13_ON_L      = 0x3a,
        LED13_ON_H      = 0x3b,
        LED13_OFF_L     = 0x3c,
        LED13_OFF_H     = 0x3d,
        LED14_ON_L      = 0x3e,
        LED14_ON_H      = 0x3f,
        LED14_OFF_L     = 0x40,
        LED14_OFF_H     = 0x41,
        LED15_ON_L      = 0x42,
        LED15_ON_H      = 0x43,
        LED15_OFF_L     = 0x44,
        LED15_OFF_H     = 0x45,
        ALL_LED_ON_L    = 0xFA,
        ALL_LED_ON_H    = 0xFB,
        ALL_LED_OFF_L   = 0xFC,
        ALL_LED_OFF_H   = 0xFD,
        PRE_SCALE       = 0xFE,
        TestMode        = 0xFF
    };

    enum RegisterMasks : uint8_t {
        MODE1_RESTART   = BIT7,
        MODE1_EXTCLK    = BIT6,
        MODE1_AI        = BIT5,
        MODE1_SLEEP     = BIT4,
        MODE1_SUB1      = BIT3,
        MODE1_SUB2      = BIT2,
        MODE1_SUB3      = BIT1,
        MODE1_ALLCALL   = BIT0,

        MODE2_INVRT     = BIT4,
        MODE2_OCH       = BIT3,
        MODE2_OUTDRV    = BIT2,
        MODE2_OUTNE_H   = BIT1,
        MODE2_OUTNE_L   = BIT0
    };

    PCA9685(uint8_t bus = 7, uint8_t address=0x40);
    PCA9685(const PCA9685&) = delete;
    PCA9685(PCA9685&&) = default;
    ~PCA9685();

    PCA9685& operator=(const PCA9685&) = delete;
    PCA9685& operator=(PCA9685&&) = default;

    // attempt to connect to the pca9685
    bool Open();

    // invalidate all pwms and close the connection with the pca9685
    void Close();

    // invalidate all pwms and restart the pca9685
    void Reset(); 

    // change the output pwm frequency between 24 and 1526 Hz
    void setFrequency(uint16_t frequency);

    void setAllPWM(uint16_t on_counts, uint16_t off_counts);
    void setAllOff();
    void setAllOn();

    // set the duty cycle of a channel, duty in [0,1]
    void setDuty(uint8_t channel, float duty, float delay=0.0f);

    // set the output by us
    void setUS(uint8_t channel, uint16_t us, uint16_t delay_us=0);

    void setOn(uint8_t channel);
    void setOff(uint8_t channel);
private:
    int i2c_fd;
    uint8_t i2c_address;
    uint8_t i2c_bus;

    float clock_frequency;
    float clock_us;
    uint16_t output_frequency;
};

#endif