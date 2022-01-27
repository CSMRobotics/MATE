#ifndef CSMUTIL_BMR4800116_005_HPP
#define CSMUTIL_BMR4800116_005_HPP

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include <limits.h>
#include <iostream>
#include <cstring>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#define BMR4800116_005_ADDRESS                             (0x55)

// registers
#define BMR4800116_005_OPERATION                           (0x01)
#define BMR4800116_005_ON_OFF_CONFIG                       (0x02)
#define BMR4800116_005_CLEAR_FAULTS                        (0x03)
#define BMR4800116_005_WRITE_PROTECT                       (0x10)
#define BMR4800116_005_STORE_DEFAULT_ALL                   (0x11)
#define BMR4800116_005_RESTORE_DEFAULT_ALL                 (0x12)
#define BMR4800116_005_STORE_USER_ALL                      (0x15)
#define BMR4800116_005_RESTORE_USER_ALL                    (0x16)
#define BMR4800116_005_CAPABILITY                          (0x19)
#define BMR4800116_005_VOUT_MODE                           (0x20)
#define BMR4800116_005_VOUT_COMMAND                        (0x21)
#define BMR4800116_005_VOUT_TRIM                           (0x22)
#define BMR4800116_005_VOUT_CAL_OFFSET                     (0x23)
#define BMR4800116_005_VOUT_MAX                            (0x24)
#define BMR4800116_005_VOUT_MARGIN_HIGH                    (0x25)
#define BMR4800116_005_VOUT_MARGIN_LOW                     (0x26)
#define BMR4800116_005_VOUT_TRANSITION_RATE                (0x27)
#define BMR4800116_005_VOUT_DROOP                          (0x28)
#define BMR4800116_005_VOUT_SCALE_LOOP                     (0x29)
#define BMR4800116_005_VOUT_SCALE_MONITOR                  (0x2A)
#define BMR4800116_005_MAX_DUTY                            (0x32)
#define BMR4800116_005_FREQUENCY_SWITCH                    (0x33)
#define BMR4800116_005_VIN_ON                              (0x35)
#define BMR4800116_005_VIN_OFF                             (0x36)
#define BMR4800116_005_INTERLEAVE                          (0x37)
#define BMR4800116_005_IOUT_CAL_OFFSET                     (0x39)
#define BMR4800116_005_VOUT_OV_FAULT_LIMIT                 (0x40)
#define BMR4800116_005_VOUT_OV_FAULT_RESPONSE              (0x41)
#define BMR4800116_005_VOUT_OV_WARN_LIMIT                  (0x42)
#define BMR4800116_005_VOUT_UV_WARN_LIMIT                  (0x43)
#define BMR4800116_005_VOUT_UV_FAULT_LIMIT                 (0x44)
#define BMR4800116_005_VOUT_UV_FAULT_RESPONSE              (0x45)
#define BMR4800116_005_IOUT_OC_FAULT_LIMIT                 (0x46)
#define BMR4800116_005_IOUT_OC_FAULT_RESPONSE              (0x47)
#define BMR4800116_005_IOUT_OC_LV_FAULT_LIMIT              (0x48)
#define BMR4800116_005_IOUT_OC_WARN_LIMIT                  (0x4A)
#define BMR4800116_005_OT_FAULT_LIMIT                      (0x4F)
#define BMR4800116_005_OT_FAULT_RESPONSE                   (0x50)
#define BMR4800116_005_OT_WARN_LIMIT                       (0x51)
#define BMR4800116_005_UT_WARN_LIMIT                       (0x52)
#define BMR4800116_005_UT_FAULT_LIMIT                      (0x53)
#define BMR4800116_005_UT_FAULT_RESPONSE                   (0x54)
#define BMR4800116_005_VIN_OV_FAULT_LIMIT                  (0x55)
#define BMR4800116_005_VIN_OV_FAULT_RESPONSE               (0x56)
#define BMR4800116_005_VIN_OV_WARN_LIMIT                   (0x57)
#define BMR4800116_005_VIN_UV_WARN_LIMIT                   (0x58)
#define BMR4800116_005_VIN_UV_FAULT_LIMIT                  (0x59)
#define BMR4800116_005_VIN_UV_FAULT_RESPONSE               (0x5A)
#define BMR4800116_005_POWER_GOOD_ON                       (0x5E)
#define BMR4800116_005_POWER_GOOD_OFF                      (0x5F)
#define BMR4800116_005_TON_DELAY                           (0x60)
#define BMR4800116_005_TON_RISE                            (0x61)
#define BMR4800116_005_TON_MAX_FAULT_LIMIT                 (0x62)
#define BMR4800116_005_TON_MAX_FAULT_RESPONSE              (0x63)
#define BMR4800116_005_TOFF_DELAY                          (0x64)
#define BMR4800116_005_TOFF_FALL                           (0x65)
#define BMR4800116_005_TOFF_MAX_WARN_LIMIT                 (0x66)
#define BMR4800116_005_STATUS_BYTE                         (0x78)
#define BMR4800116_005_STATUS_WORD                         (0x79)
#define BMR4800116_005_STATUS_VOUT                         (0x7A)
#define BMR4800116_005_STATUS_IOUT                         (0x7B)
#define BMR4800116_005_STATUS_INPUT                        (0x7C)
#define BMR4800116_005_STATUS_TEMPERATURE                  (0x7D)
#define BMR4800116_005_STATUS_CML                          (0x7E)
#define BMR4800116_005_READ_VIN                            (0x88)
#define BMR4800116_005_READ_VOUT                           (0x8B)
#define BMR4800116_005_READ_IOUT                           (0x8C)
#define BMR4800116_005_READ_TEMPERATURE_1                  (0x8D)
#define BMR4800116_005_READ_TEMPERATURE_2                  (0x8E)
#define BMR4800116_005_READ_DUTY_CYCLE                     (0x94)
#define BMR4800116_005_READ_FREQUENCY                      (0x95)
#define BMR4800116_005_PMBUS_REVISION                      (0x98)
#define BMR4800116_005_MFR_ID                              (0x99)
#define BMR4800116_005_MFR_MODEL                           (0x9A)
#define BMR4800116_005_MFR_REVISION                        (0x9B)
#define BMR4800116_005_MFR_LOCATION                        (0x9C)
#define BMR4800116_005_MFR_DATE                            (0x9D)
#define BMR4800116_005_MFR_SERIAL                          (0x9E)
#define BMR4800116_005_USER_DATA_00                        (0xB0)
#define BMR4800116_005_MFR_VIN_OV_WARN_RESPONSE            (0xC4)
#define BMR4800116_005_MFR_CONFIG_UNUSED_PINS              (0xC5)
#define BMR4800116_005_MFR_RC_LEVEL                        (0xC6)
#define BMR4800116_005_MFR_KS_PRETRIG                      (0xC7)
#define BMR4800116_005_MFR_FAST_VIN_OFF_OFFSET             (0xC8)
#define BMR4800116_005_MFR_PGOOD_POLARITY                  (0xD0)
#define BMR4800116_005_MFR_FAST_OCP_CFG                    (0xD1)
#define BMR4800116_005_MFR_RESPONSE_UNIT_CFG               (0xD2)
#define BMR4800116_005_MFR_VIN_SCALE_MONITOR               (0xD3)
#define BMR4800116_005_MFR_PREBIAS_DVDT_CFG                (0xD4)
#define BMR4800116_005_MFR_FILTER_SELECT                   (0xD5)
#define BMR4800116_005_MFR_GET_SNAPSHOT                    (0xD7)
#define BMR4800116_005_MFR_TEMP_COMPENSATION               (0xD8)
#define BMR4800116_005_MFR_SET_ROM_MODE                    (0xD9)
#define BMR4800116_005_MFR_ISHARE_THRESHOLD                (0xDA)
#define BMR4800116_005_MFR_GET_RAMP_DATA                   (0xDB)
#define BMR4800116_005_MFR_SELECT_TEMPERATURE_SENSOR       (0xDC)
#define BMR4800116_005_MFR_VIN_OFFSET                      (0xDD)
#define BMR4800116_005_MFR_VOUT_OFFSET_MONITOR             (0xDE)
#define BMR4800116_005_MFR_GET_STATUS_DATA                 (0xDF)
#define BMR4800116_005_MFR_SPECIAL_OPTIONS                 (0xE0)
#define BMR4800116_005_MFR_TEMP_OFFSET_INT                 (0xE1)
#define BMR4800116_005_MFR_REMOTE_TEMP_CAL                 (0xE2)
#define BMR4800116_005_MFR_REMOTE_CTRL                     (0xE3)
#define BMR4800116_005_MFR_VFF_PARAMS                      (0xE6)
#define BMR4800116_005_MFR_TEMP_COEFF                      (0xE7)
#define BMR4800116_005_MFR_FILTER_COEFF                    (0xE8)
#define BMR4800116_005_MFR_FILTER_NLR_GAIN                 (0xE9)
#define BMR4800116_005_MFR_MIN_DUTY                        (0xEB)
#define BMR4800116_005_MFR_ACTIVE_CLAMP                    (0xEC)
#define BMR4800116_005_MFR_OFFSET_ADDRES                   (0xEE)
#define BMR4800116_005_MFR_DBV_CONFIG                      (0xEF)
#define BMR4800116_005_MFR_DEBUG_BUFF                      (0xF0)
#define BMR4800116_005_MFR_SETUP_PASSWORD                  (0xF1)
#define BMR4800116_005_MFR_DISABLE_SECURITY_ONCE           (0xF2)
#define BMR4800116_005_MFR_SECURITY_BIT_MASK               (0xF4)
#define BMR4800116_005_MFR_TRANSFORMER_TURN                (0xF5)
#define BMR4800116_005_MFR_OSC_TRIM                        (0xF6)
#define BMR4800116_005_MFR_DLC_CONFIG                      (0xF7)
#define BMR4800116_005_MFR_ILIM_SOFTSTART                  (0xF8)
#define BMR4800116_005_MFR_MULTI_PIN_CONFIG                (0xF9)
#define BMR4800116_005_MFR_ADDED_DROOP_DURING_RAMP         (0xFC)
#define BMR4800116_005_MFR_FIRMWARE_DATA                   (0xFD)
#define BMR4800116_005_MFR_RESTAR                          (0xFE)

struct PowerStats {
    uint16_t inputVoltage;
    uint16_t outputVoltage;
    uint16_t outputCurrent;
    uint16_t temp1;
    uint16_t temp2;
};

class BMR4800116_005 {
public:
    BMR4800116_005(uint8_t operationMode=0x84);

    PowerStats getPowerStats();
private:
    int m_fd;

    uint16_t readWord(uint8_t reg);
    uint8_t readByte(uint8_t reg);
    bool writeByte(uint8_t reg, uint8_t value);
    bool writeWord(uint8_t reg, uint16_t value);
};

#endif // CSMUTIL_BMR4800116_005_HPP