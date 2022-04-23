#ifndef BNO055_HPP
#define BNO055_HPP

#include <unistd.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include <limits.h>
#include <iostream>
#include <cstring>

#include <eigen3/Eigen/Dense>

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

// TODO: finish documentation

#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)

#define NUM_BNO055_OFFSET_REGISTERS (22)

enum BNO055_Registers_Page_0 {
    /* REGISTER           VALUE      DESCRIPTION                  BITS      DEFAULT                         ACCESS (Read/Write) */
    CHIP_ID             = (0x00), // BNO055 Chip ID              <7:0>       0xA0                             R
    ACC_ID              = (0x01), // ACC Chip ID                 <7:0>       0xFB                             R
    MAG_ID              = (0x02), // MAG Chip ID                 <7:0>       0x32                             R
    GYR_ID              = (0x03), // GYRO Chip ID                <7:0>       0x0F                             R
    SW_REV_ID_LSB       = (0x04), // SW Revision ID              <7:0>       0x08 (subject to change)         R
    SW_REV_ID_MSB       = (0x05), // SW Revision ID              <15:8>      0x03 (subject to change)         R
    BL_REV_ID           = (0x06), // Bootloader Version          <7:0>        NA                              R
    PAGE_ID             = (0x07), // Page ID                     <7:0>       0x00                             RW

    ACC_DATA_X_LSB      = (0x08), // Acceleration Data X         <7:0>       0x00                             R
    ACC_DATA_X_MSB      = (0x09), // Acceleration Data X         <15:8>      0x00                             R
    ACC_DATA_Y_LSB      = (0x0A), // Acceleration Data Y         <7:0>       0x00                             R
    ACC_DATA_Y_MSB      = (0X0B), // Acceleration Data Y         <15:8>      0x00                             R
    ACC_DATA_Z_LSB      = (0x0C), // Acceleration Data Z         <7:0>       0x00                             R
    ACC_DATA_Z_MSB      = (0x0D), // Acceleration Data Z         <15:8>      0x00                             R

    MAG_DATA_X_LSB      = (0x0E), // Magnetometer Data X         <7:0>       0x00                             R
    MAG_DATA_X_MSB      = (0x0F), // Magnetometer Data X         <15:8>      0x00                             R
    MAG_DATA_Y_LSB      = (0x10), // Magnetometer Data Y         <7:0>       0x00                             R
    MAG_DATA_Y_MSB      = (0x11), // Magnetometer Data Y         <15:8>      0x00                             R
    MAG_DATA_Z_LSB      = (0x12), // Magnetometer Data Z         <7:0>       0x00                             R
    MAG_DATA_Z_MSB      = (0x13), // Magnetometer Data Z         <15:8>      0x00                             R

    GYR_DATA_X_LSB      = (0x14), // Gyroscope Data X            <7:0>       0x00                             R
    GYR_DATA_X_MSB      = (0x15), // Gyroscope Data X            <15:8>      0x00                             R
    GYR_DATA_Y_LSB      = (0x16), // Gyroscope Data Y            <7:0>       0x00                             R
    GYR_DATA_Y_MSB      = (0x17), // Gyroscope Data Y            <15:8>      0x00                             R
    GYR_DATA_Z_LSB      = (0x18), // Gyroscope Data Z            <7:0>       0x00                             R
    GYR_DATA_Z_MSB      = (0x19), // Gyroscope Data Z            <15:8>      0x00                             R

    EUL_HEADING_LSB     = (0x1A), // Heading Data                <7:0>       0x00                             R
    EUL_HEADING_MSB     = (0x1B), // Heading Data                <15:8>      0x00                             R
    EUL_ROLL_LSB        = (0x1C), // Roll Data                   <7:0>       0x00                             R
    EUL_ROLL_MSB        = (0x1D), // Roll Data                   <15:8>      0x00                             R
    EUL_PITCH_LSB       = (0x1E), // Pitch Data                  <7:0>       0x00                             R
    EUL_PUTCH_MSB       = (0x1F), // Pitch Data                  <15:8>      0x00                             R

    QUA_DATA_W_LSB      = (0x20), // Quaternion W Data           <7:0>       0x00                             R
    QUA_DATA_W_MSB      = (0x21), // Quaternion W Data           <15:8>      0x00                             R
    QUA_DATA_X_LSB      = (0x22), // Quaternion X Data           <7:0>       0x00                             R
    QUA_DATA_X_MSB      = (0x23), // Quaternion X Data           <15:8>      0x00                             R
    QUA_DATA_Y_LSB      = (0x24), // Quaternion Y Data           <7:0>       0x00                             R
    QUA_DATA_Y_MSB      = (0x25), // Quaternion Y Data           <15:8>      0x00                             R
    QUA_DATA_Z_LSB      = (0x26), // Quaternion Z Data           <7:0>       0x00                             R
    QUA_DATA_Z_MSB      = (0x27), // Quaternion Z Data           <15:8>      0x00                             R

    LIA_DATA_X_LSB      = (0x28), // Linear Accleration Data X   <7:0>       0x00                             R
    LIA_DATA_X_MSB      = (0x29), // Linear Accleration Data X   <15:8>      0x00                             R
    LIA_DATA_Y_LSB      = (0x2A), // Linear Accleration Data Y   <7:0>       0x00                             R
    LIA_DATA_Y_MSB      = (0x2B), // Linear Accleration Data Y   <15:8>      0x00                             R
    LIA_DATA_Z_LSB      = (0x2C), // Linear Accleration Data Z   <7:0>       0x00                             R
    LIA_DATA_Z_MSB      = (0x2D), // Linear Accleration Data Z   <15:8>      0x00                             R

    GRV_DATA_X_LSB      = (0x2E), // Gravity Vector Data X       <7:0>       0x00                             R
    GRV_DATA_X_MSB      = (0x2F), // Gravity Vector Data X       <15:8>      0x00                             R
    GRV_DATA_Y_LSB      = (0x30), // Gravity Vector Data X       <7:0>       0x00                             R
    GRV_DATA_Y_MSB      = (0x31), // Gravity Vector Data X       <15:8>      0x00                             R
    GRV_DATA_Z_LSB      = (0x32), // Gravity Vector Data X       <7:0>       0x00                             R
    GRV_DATA_Z_MSB      = (0x33), // Gravity Vector Data X       <15:8>      0x00                             R

    TEMP                = (0x34), // Temperature Data            <7:0>       0x00                             R


    // More complicated registers
    /* REGISTER           VALUE        PURPOSE                                                 BITS 7->0                                                                              DEFAULT                         ACCESS */
    /*                                                        |      7      |      6      |      5      |      4      |      3      |      2      |      1      |      0      |*/
    CALIB_STAT          = (0x35), // Calibration Status 0->3  |    SYS Calib Status       |    GYR Calib Status       |    ACC Calib Status       |    MAG Calib Status       |         0x00                            R
    ST_RESULT           = (0x36), //                          |                                                       |   ST_MCU    |   ST_GYR    |   ST_MAG    |   ST_ACC    |         0x0F                            R
    INT_STA             = (0x37), //                          |  ACC_N_M    |  ACC_A_M    | ACC_HIGH_G  |             |GYR_HIGH_RATE  | GYRO_A_M  |             |             |         0x00                            R
    SYS_CLK_STATUS      = (0x38), //                          |                                                                                                 | ST_MAIN_CLK |         0x00                            RW
    SYS_STATUS          = (0x39), //                          |                                              System Status Code                                               |         0x00                            R
    SYS_ERR             = (0x3A), //                          |                                              System Error Code                                                |         0x00                            R
    UNIT_SEL            = (0x3B), //                          |ORI_ANDRID_WIN|            |  TEMP_UNIT  |             |             |  EUL_UNIT   |  GYR_UNIT   |  ACC_UNIT   |         0x80                            RW
    
    // 0x3C is reserved

    OPR_MODE            = (0x3D), //                          |             |             |             |             |                     Operation Mode                    |         0x1C                            RW
    PWR_MODE            = (0x3E), //                          |             |             |             |             |             |             |        Power Mode         |         0x00                            RW
    SYS_TRIGGER         = (0x3F), //                          |   CLK_SEL   |   RST_INT   |   RST_SYS   |             |             |             |             |  Self_Test  |         0x00                            RW
    TEMP_SOURCE         = (0x40), //                          |             |             |             |             |             |             |        TEMP_Source        |         0x02                            RW
    AXIS_MAP_CONFIG     = (0x41), //   Remapped Axis Values   |             |             |   Remapped Z Axis Value   |   Remapped Y Axis Value   |   Remapped X Axis Value   |          TBD                            RW
    AXIS_MAP_SIGN       = (0x42), //   Remapped Axis Signs    |             |             |             |             |             |Remap X Axis+|Remap Y Axis+|Remap Z Axis+|          TBD                            RW

    // 0x43 - 0x54 are reserved

    /* REGISTER           VALUE      DESCRIPTION                  BITS      DEFAULT                         ACCESS (Read/Write) */
    ACC_OFFSET_X_LSB    = (0x55), // Accelerometer Offset X      <7:0>       0x00                            RW
    ACC_OFFSET_X_MSB    = (0x56), // Accelerometer Offset X      <15:8>      0x00                            RW
    ACC_OFFSET_Y_LSB    = (0x57), // Accelerometer Offset Y      <7:0>       0x00                            RW
    ACC_OFFSET_Y_MSB    = (0x58), // Accelerometer Offset Y      <15:8>      0x00                            RW
    ACC_OFFSET_Z_LSB    = (0x59), // Accelerometer Offset Z      <7:0>       0x00                            RW
    ACC_OFFSET_Z_MSB    = (0x5A), // Accelerometer Offset Z      <15:8>      0x00                            RW

    MAG_OFFSET_X_LSB    = (0x5B), // Magnetometer Offset X       <7:0>       0x00                            RW
    MAG_OFFSET_X_MSB    = (0x5C), // Magnetometer Offset X       <15:8>      0x00                            RW
    MAG_OFFSET_Y_LSB    = (0x5D), // Magnetometer Offset Y       <7:0>       0x00                            RW
    MAG_OFFSET_Y_MSB    = (0x5E), // Magnetometer Offset Y       <15:8>      0x00                            RW
    MAG_OFFSET_Z_LSB    = (0x5F), // Magnetometer Offset Z       <7:0>       0x00                            RW
    MAG_OFFSET_Z_MSB    = (0x60), // Magnetometer Offset Z       <15:8>      0x00                            RW

    GYR_OFFSET_X_LSB    = (0x61), // Gyroscope Offset X          <7:0>       0x00                            RW
    GYR_OFFSET_X_MSB    = (0x62), // Gyroscope Offset X          <15:8>      0x00                            RW
    GYR_OFFSET_Y_LSB    = (0x63), // Gyroscope Offset Y          <7:0>       0x00                            RW
    GYR_OFFSET_Y_MSB    = (0x64), // Gyroscope Offset Y          <15:8>      0x00                            RW
    GYR_OFFSET_Z_LSB    = (0x65), // Gyroscope Offset Z          <7:0>       0x00                            RW
    GYR_OFFSET_Z_MSB    = (0x66), // Gyroscope Offset Z          <15:8>      0x00                            RW

    ACC_RADIUS_LSB      = (0x67), // Accelerometer Radius        <7:0>        --                             RW
    ACC_RADIUS_MSB      = (0x68), // Accelerometer Radius        <7:0>        --                             RW

    MAG_RADIUS_LSB      = (0x69), // Magnetometer Radius         <7:0>        --                             RW
    MAG_RADIUS_MSB      = (0x6A), // Magnetometer Radius         <7:0>        --                             RW
};


enum BNO055_Registers_Page_1 {
    /* REGISTER           VALUE        PURPOSE                                                 BITS 7->0                                                                              DEFAULT                         ACCESS (Read/Write) */
    /*                                                        |      7      |      6      |      5      |      4      |      3      |      2      |      1      |      0      |*/
    //PAGE_ID             = (0x07), // Page ID                |                                                    PAGE ID                                                    |         0x01                            R
    ACC_CONFIG          = (0x08), //                          |               ACC_PWR_MODE              |                  ACC_BW                 |        ACC_RANGE          |         0x0D                            R
    MAG_CONFIG          = (0x09), //                          |             |      MAG_POWER_MODE       |       MAG_OPR_MODE        |          MAG_DATA_OUTPUT_RATE           |         0x6D                            R
    GYR_CONFIG_0        = (0x0A), //                          |                           |              GYR_BANDWIDTH              |                GYR_RANGE                |         0x38                            R
    GYR_CONFIG_1        = (0x0B), //                          |             |                                                       |              GYR_POWER_MODE             |         0x00                            R
    ACC_SLEEP_CONFIG    = (0x0C), //                          |                                         |                     SLP_DURATION                      |  SLP_MODE   |         0x00                            R
    GYR_SLEEP_CONFIG    = (0x0D), //                          |                           |            AUTO_SLP_DURATION            |              SLP_DURATION               |         0x00                            R

    // 0x0E is reserved

    INT_MSK             = (0x0F), //                          |   ACC_NM    |   ACC_AM    | ACC_HIGH_G  |             |GYR_HIGH_RATE|   GYRO_AM   |             |             |         0x00                            R
    INT_EN              = (0x10), //                          |   ACC_NM    |   ACC_AM    | ACC_HIGH_G  |             |GYR_HIGH_RATE|   GYRO_AM   |             |             |         0x00                            R
    ACC_AM_THRES        = (0x11), //                          |                                      Accelerometer Any motion threshold                                       |         0x14                            R
    ACC_INT_SETTINGS    = (0x12), //                          |  HG_Z_AXIS  |  HG_Y_AXIS  |  HG_X_AXIS  | AM/NM_Z_AXIS| AM/NM_Y_AXIS| AM/NM_X_AXIS|          AM_DUR           |         0x03                            R
    ACC_HG_DURATION     = (0x13), //                          |                                        Accelerometer High G Duration                                          |         0x0F                            R
    ACC_HG_THRES        = (0x14), //                          |                                        Accelerometer High G Threshold                                         |         0xC0                            R
    ACC_NM_THRES        = (0x15), //                          |                                    Accelerometer NO/SLOW motion threshold                                     |         0x0A                            R
    ACC_NM_SET          = (0x16), //                          |             |                              NO/SLOW Motion Duration                              |    SMNM     |         0x0B                            R
    GYR_INT_SETTINGS    = (0x17), //                          |   HR_FILT   |   AM_FILT   |  HR_Z_AXIS  |  HR_Y_AXIS  |  HR_X_AXIS  |  AM_Z_AXIS  |  AM_Y_AXIS  |  AM_X_AXIS  |         0x00                            R
    GYR_HR_X_SET        = (0x18), //                          |             |      HR_X_THRES_HYST      |                            HR_X_Threshold                           |         0x01                            R
    GYR_DUR_X           = (0x19), //                          |                                                 HR_Y_Duration                                                 |         0x19                            R
    GYR_HR_Y_SET        = (0x1A), //                          |             |      HR_Y_THRES_HYST      |                            HR_Y_Threshold                           |         0x01                            R
    GYR_DUR_Y           = (0x1B), //                          |                                                 HR_Z_Duration                                                 |         0x19                            R
    GYR_HR_Z_SET        = (0x1C), //                          |             |      HR_Z_THRES_HYST      |                            HR_Z_Threshold                           |         0x01                            R
    GYR_DUR_Z           = (0x1D), //                          |                                                 HR_Z_Duration                                                 |         0x19                            R
    GYR_AM_THRES        = (0x1E), //                          |             |                                    Gyro Any Motion Threshold                                    |         0x04                            R
    GYR_AM_SET          = (0x1F), //                          |             |             |             |             |       Awake Duration      |       Slope Samples       |         0x0A                            R
                                                            /*|      7      |      6      |      5      |      4      |      3      |      2      |      1      |      0      |*/

    // 0x20 - 0x4F are reserved

    /* REGISTER           VALUE      DESCRIPTION                  BITS      DEFAULT                         ACCESS (Read/Write) */
    UNIQUE_ID_0x50      = (0x50), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x51      = (0x51), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x52      = (0x52), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x53      = (0x53), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x54      = (0x54), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x55      = (0x55), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x56      = (0x56), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x57      = (0x57), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x58      = (0x58), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x59      = (0x59), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x5A      = (0x5A), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x5B      = (0x5B), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x5C      = (0x5C), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x5D      = (0x5D), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x5E      = (0x5E), // BNO Unique ID               <7:0>        NA                              R
    UNIQUE_ID_0x5F      = (0x5F), // BNO Unique ID               <7:0>        NA                              R

    // 0x60 - 0x7F are reserved
};

enum BNO055_Powermodes {
    NORMAL              = 0X00,
    LOWPOWER            = 0X01,
    SUSPEND             = 0X02,
};

enum BNO055_Opmode {
    CONFIG              = (0X00),
    ACCONLY             = (0X01),
    MAGONLY             = (0X02),
    GYRONLY             = (0X03),
    ACCMAG              = (0X04),
    ACCGYRO             = (0X05),
    MAGGYRO             = (0X06),
    AMG                 = (0X07),
    IMUPLUS             = (0X08),
    COMPASS             = (0X09),
    M4G                 = (0X0A),
    NDOF_FMC_OFF        = (0X0B),
    NDOF                = (0X0C),
};

enum BNO055_Axis_Remap_Config {
    CONFIG_P0                  = (0x21),
    CONFIG_P1                  = (0x24), // default
    CONFIG_P2                  = (0x24),
    CONFIG_P3                  = (0x21),
    CONFIG_P4                  = (0x24),
    CONFIG_P5                  = (0x21),
    CONFIG_P6                  = (0x21),
    CONFIG_P7                  = (0x24)
};

enum BNO055_Axis_Remap_Sign {
    SIGN_P0                  = (0x04),
    SIGN_P1                  = (0x00), // default
    SIGN_P2                  = (0x06),
    SIGN_P3                  = (0x02),
    SIGN_P4                  = (0x03),
    SIGN_P5                  = (0x01),
    SIGN_P6                  = (0x07),
    SIGN_P7                  = (0x05)
};

enum Vector_Type
{
    ACCELEROMETER = BNO055_Registers_Page_0::ACC_DATA_X_LSB,
    MAGNETOMETER  = BNO055_Registers_Page_0::MAG_DATA_X_LSB,
    GYROSCOPE     = BNO055_Registers_Page_0::GYR_DATA_X_LSB,
    EULER         = BNO055_Registers_Page_0::EUL_HEADING_LSB,
    LINEARACCEL   = BNO055_Registers_Page_0::LIA_DATA_X_LSB,
    GRAVITY       = BNO055_Registers_Page_0::GRV_DATA_X_LSB
};

struct BNO055_REV_Info_t {
    uint8_t  accel_rev;
    uint8_t  mag_rev;
    uint8_t  gyro_rev;
    uint16_t sw_rev;
    uint8_t  bl_rev;
};

struct BNO055_Offsets_t {
    uint16_t accel_offset_x;
    uint16_t accel_offset_y;
    uint16_t accel_offset_z;
    uint16_t gyro_offset_x;
    uint16_t gyro_offset_y;
    uint16_t gyro_offset_z;
    uint16_t mag_offset_x;
    uint16_t mag_offset_y;
    uint16_t mag_offset_z;

    uint16_t accel_radius;
    uint16_t mag_radius;
};

class BNO055 {
public:
    BNO055(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A);
    bool begin(BNO055_Opmode mode = BNO055_Opmode::NDOF);
    void setMode(BNO055_Opmode mode);
    void getRevInfo(BNO055_REV_Info_t* info);
    void displayRevInfo();
    void setExtCrystalUse(bool usextal);
    void getSystemStatus(uint8_t* sysStatus, uint8_t* selfTestResult, uint8_t* sysError);
    void displaySystemStatus();
    void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag);

    
    Eigen::Vector3d getVector(Vector_Type type);
    Eigen::Quaterniond getQuat();
    int8_t getTemp();

    bool getSensorOffsets(uint8_t* calibData);
    bool getSensorOffsets(BNO055_Offsets_t& offsetType);
    void setSensorOffsets(const uint8_t* calibData);
    void setSensorOffsets(const BNO055_Offsets_t& offsetType);
    bool isFullyCalibrated();

    int m_fdBNO;
    int m_i2cChannel;
private:
    uint8_t read8(uint8_t reg);
    bool readLen(uint8_t reg, uint8_t* buf, uint8_t len);
    bool write8(uint8_t reg, uint8_t value);

    uint8_t m_address;
    uint8_t m_sensorID;
    BNO055_Opmode m_mode;
    
};

#endif //BNO055_HPP