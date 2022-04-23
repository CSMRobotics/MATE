#include "bno055/CSMBNO055.hpp"

/*
* Construct new BNO055 with specified ID and at an I2C address
*/
BNO055::BNO055(int32_t sensorID, uint8_t address) {
    m_sensorID = sensorID;
    m_address = address;
    m_i2cChannel = 1;
    
    const char* filename = "/dev/i2c-1";
    m_fdBNO = open(filename, O_RDWR);
    if(m_fdBNO < 0) {
        perror("Failed to open I2c bus file");
        exit(1);
    }
    if(ioctl(m_fdBNO, I2C_SLAVE, address) < 0) {
        perror("Failed to open i2c bus");
        exit(1);
    }
}

/*
* Setup default registers to perform in given Opmode. Default = NDOF
* Call this only once per BNO055.
*/
bool BNO055::begin(BNO055_Opmode mode) {
    // ensure we have right device
    uint8_t id = read8(BNO055_Registers_Page_0::CHIP_ID);
    if(id != BNO055_ID) {
        usleep(1000000); // try to wait for boot
        id = read8(BNO055_Registers_Page_0::CHIP_ID);
        if(id != BNO055_ID) {
            return false; // if boot does not happen, cry about it
        }
    }

    // ensure we are in correct opmode
    setMode(BNO055_Opmode::CONFIG);

    // reset
    write8(BNO055_Registers_Page_0::SYS_TRIGGER, 0x20);
    while(read8(BNO055_Registers_Page_0::CHIP_ID) != BNO055_ID) {
        usleep(10000);
    }
    usleep(50000);

    write8(BNO055_Registers_Page_0::PWR_MODE, BNO055_Powermodes::NORMAL);
    usleep(10000);

    write8(BNO055_Registers_Page_0::PAGE_ID, 0);

    write8(BNO055_Registers_Page_0::SYS_TRIGGER, 0x0);
    usleep(10000);

    // Set requested mode
    setMode(mode);
    usleep(20000);

    return true;
}

/*
* Change current operational mode
*/
void BNO055::setMode(BNO055_Opmode mode) {
    m_mode = mode;
    write8(BNO055_Registers_Page_0::OPR_MODE, mode);
    usleep(30000);
}

/*
* Set BNO055 to use external clock
*/
void BNO055::setExtCrystalUse(bool usextal) {
    BNO055_Opmode modeBkp = m_mode;

    setMode(BNO055_Opmode::CONFIG); // ensure we are in config mode
    usleep(25000);
    write8(BNO055_Registers_Page_0::PAGE_ID, 0); // ensure we are in correct page
    if(usextal) { // write the correct value for usextal into sys_trigger
        write8(BNO055_Registers_Page_0::SYS_TRIGGER, 0x80);
    } else {
        write8(BNO055_Registers_Page_0::SYS_TRIGGER, 0x00);
    }
    usleep(10000);

    setMode(modeBkp); // return to previous mode
    usleep(20000);
}

/*
* Perform a self test and write result into pointers
*/
void BNO055::getSystemStatus(uint8_t* sysStatus, uint8_t* selfTestResult, uint8_t* sysError) {
    write8(BNO055_Registers_Page_0::PAGE_ID, 0); // ensure we are on correct page

    if(sysStatus != nullptr) {
        *sysStatus = read8(BNO055_Registers_Page_0::SYS_STATUS);
    }

    if(selfTestResult != nullptr) {
        *selfTestResult = read8(BNO055_Registers_Page_0::ST_RESULT);
    }

    if(sysError != nullptr) {
        *sysError = read8(BNO055_Registers_Page_0::SYS_ERR);
    }

    usleep(200000);
}

/*
* Get information about current chip and software revisions and write into struct pointer
*/
void BNO055::getRevInfo(BNO055_REV_Info_t* info) {
    uint8_t a, b;
    memset(info, 0, sizeof(BNO055_REV_Info_t));
    info->accel_rev = read8(BNO055_Registers_Page_0::ACC_ID);
    info->mag_rev = read8(BNO055_Registers_Page_0::MAG_ID);
    info->gyro_rev = read8(BNO055_Registers_Page_0::GYR_ID);
    info->bl_rev = read8(BNO055_Registers_Page_0::BL_REV_ID);

    a = read8(BNO055_Registers_Page_0::SW_REV_ID_LSB);
    b = read8(BNO055_Registers_Page_0::SW_REV_ID_MSB);
    info->sw_rev = ((static_cast<uint16_t>(b) << 8) | static_cast<uint16_t>(a));
}

/*
* Get calibration data for sys, gyroscope, accelerometer, and magnetometer
* If all data == 3, fully calibrated
*/
void BNO055::getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
    uint8_t calData = read8(BNO055_Registers_Page_0::CALIB_STAT);
    if(sys != nullptr) {
        *sys = (calData >> 6) & 0x03;
    }
    if(gyro != nullptr) {
        *gyro = (calData >> 4) & 0x03;
    }
    if(accel != nullptr) {
        *accel = (calData >> 2) & 0x03;
    }
    if(mag != nullptr) {
        *mag = calData & 0x03;
    }
}

/*
* Get the current temperature
*/
int8_t BNO055::getTemp() {
    int8_t temp = static_cast<int8_t>(read8(BNO055_Registers_Page_0::TEMP));
    return temp;
}

/*
* Read registers that correspond to specified Vector_Type
*/
Eigen::Vector3d BNO055::getVector(Vector_Type type) {
    uint8_t buffer[6];
    memset(buffer, 0, 6);

    int16_t i = 0, j = 0, k = 0;

    readLen(type, buffer, 6); // read vector registers

    // combine 8 bit registers into 16 bit signed short
    i = static_cast<int16_t>(buffer[0]) | (static_cast<int16_t>(buffer[1]) << 8);
    j = static_cast<int16_t>(buffer[2]) | (static_cast<int16_t>(buffer[3]) << 8);
    k = static_cast<int16_t>(buffer[4]) | (static_cast<int16_t>(buffer[5]) << 8);

    /*
    * Convert short data into correct units (SI)
    * Magic numbers come from data sheet
    */
    switch(type) {
        case Vector_Type::MAGNETOMETER:
            return Eigen::Vector3d(static_cast<double>(i) / 16.0,
                                   static_cast<double>(j) / 16.0,
                                   static_cast<double>(k) / 16.0);
        break;
        case Vector_Type::GYROSCOPE:
            return Eigen::Vector3d(static_cast<double>(i) / 900.0,
                                   static_cast<double>(j) / 900.0,
                                   static_cast<double>(k) / 900.0);
        break;
        case Vector_Type::EULER:
            return Eigen::Vector3d(static_cast<double>(i) / 16.0,
                                   static_cast<double>(j) / 16.0,
                                   static_cast<double>(k) / 16.0);
        break;
        case Vector_Type::ACCELEROMETER:
        case Vector_Type::LINEARACCEL:
        case Vector_Type::GRAVITY:
            return Eigen::Vector3d(static_cast<double>(i) / 100.0,
                                   static_cast<double>(j) / 100.0,
                                   static_cast<double>(k) / 100.0);
        break;
    }

    return Eigen::Vector3d();
}

/*
* Get the rotation quaternion for this BNO055
*/
Eigen::Quaterniond BNO055::getQuat() {
    uint8_t buffer[8];
    memset(buffer, 0, 8);

    int16_t w = 0, i = 0, j = 0, k = 0;

    readLen(BNO055_Registers_Page_0::QUA_DATA_W_LSB, buffer, 8);
    // combine 8 bit registers into 16 bit unsigned short
    w = (static_cast<uint16_t>(buffer[1]) << 8) | static_cast<uint16_t>(buffer[0]);
    i = (static_cast<uint16_t>(buffer[3]) << 8) | static_cast<uint16_t>(buffer[2]);
    j = (static_cast<uint16_t>(buffer[5]) << 8) | static_cast<uint16_t>(buffer[4]);
    k = (static_cast<uint16_t>(buffer[7]) << 8) | static_cast<uint16_t>(buffer[6]);

    // Use magic number from data sheet to convert unsigned short data to double
    const double scale = (1.0 / (1<<14));
    return Eigen::Quaterniond(scale * w, scale * i, scale * j, scale * k);
}

/*
* Read sensor offset registers into buffer
*/
bool BNO055::getSensorOffsets(uint8_t* calibData) {
    if(isFullyCalibrated()) {
        BNO055_Opmode modeBkp = m_mode;
        setMode(BNO055_Opmode::CONFIG); // ensure we are in config mode

        // read registers directly into buffer
        readLen(BNO055_Registers_Page_0::ACC_OFFSET_X_LSB, calibData, NUM_BNO055_OFFSET_REGISTERS);

        setMode(modeBkp); // return to previous mode
        return true;
    }
    return false;
}

/*
* Read sensor offset registers into struct
*/
bool BNO055::getSensorOffsets(BNO055_Offsets_t& offsetType) {
    if(isFullyCalibrated()) {
        BNO055_Opmode modeBkp = m_mode;
        setMode(BNO055_Opmode::CONFIG); // ensure we are in config mode
        usleep(25000);

        // read the registers into the sent struct
        offsetType.accel_offset_x = (read8(BNO055_Registers_Page_0::ACC_OFFSET_X_MSB) << 8) | (read8(BNO055_Registers_Page_0::ACC_OFFSET_X_MSB));
        offsetType.accel_offset_y = (read8(BNO055_Registers_Page_0::ACC_OFFSET_Y_MSB) << 8) | (read8(BNO055_Registers_Page_0::ACC_OFFSET_Y_MSB));
        offsetType.accel_offset_z = (read8(BNO055_Registers_Page_0::ACC_OFFSET_Z_MSB) << 8) | (read8(BNO055_Registers_Page_0::ACC_OFFSET_Z_MSB));

        offsetType.gyro_offset_x = (read8(BNO055_Registers_Page_0::GYR_OFFSET_X_MSB) << 8) | (read8(BNO055_Registers_Page_0::GYR_OFFSET_X_MSB));
        offsetType.gyro_offset_y = (read8(BNO055_Registers_Page_0::GYR_OFFSET_Y_MSB) << 8) | (read8(BNO055_Registers_Page_0::GYR_OFFSET_Y_MSB));
        offsetType.gyro_offset_z = (read8(BNO055_Registers_Page_0::GYR_OFFSET_Z_MSB) << 8) | (read8(BNO055_Registers_Page_0::GYR_OFFSET_Z_MSB));

        offsetType.mag_offset_x = (read8(BNO055_Registers_Page_0::MAG_OFFSET_X_MSB) << 8) | (read8(BNO055_Registers_Page_0::MAG_OFFSET_X_MSB));
        offsetType.mag_offset_y = (read8(BNO055_Registers_Page_0::MAG_OFFSET_Y_MSB) << 8) | (read8(BNO055_Registers_Page_0::MAG_OFFSET_Y_MSB));
        offsetType.mag_offset_z = (read8(BNO055_Registers_Page_0::MAG_OFFSET_Z_MSB) << 8) | (read8(BNO055_Registers_Page_0::MAG_OFFSET_Z_MSB));

        offsetType.accel_radius = (read8(BNO055_Registers_Page_0::ACC_RADIUS_MSB) << 8) | (read8(BNO055_Registers_Page_0::ACC_RADIUS_LSB));
        offsetType.mag_radius = (read8(BNO055_Registers_Page_0::MAG_RADIUS_MSB)) | (read8(BNO055_Registers_Page_0::MAG_RADIUS_LSB));

        setMode(modeBkp); // return to previous mode
        return true;
    }
    return false;
}

/*
* Write sensor offsets from buffer
*/
void BNO055::setSensorOffsets(const uint8_t* calibData) {
    BNO055_Opmode modeBkp = m_mode;
    setMode(BNO055_Opmode::CONFIG); // ensure we are in config mode
    usleep(25000);

    // write sensor offsets from buffer
    write8(BNO055_Registers_Page_0::ACC_OFFSET_X_LSB, calibData[0]);
    write8(BNO055_Registers_Page_0::ACC_OFFSET_X_MSB, calibData[1]);
    write8(BNO055_Registers_Page_0::ACC_OFFSET_Y_LSB, calibData[2]);
    write8(BNO055_Registers_Page_0::ACC_OFFSET_Y_MSB, calibData[3]);
    write8(BNO055_Registers_Page_0::ACC_OFFSET_Z_LSB, calibData[4]);
    write8(BNO055_Registers_Page_0::ACC_OFFSET_Z_MSB, calibData[5]);

    write8(BNO055_Registers_Page_0::GYR_OFFSET_X_LSB, calibData[6]);
    write8(BNO055_Registers_Page_0::GYR_OFFSET_X_MSB, calibData[7]);
    write8(BNO055_Registers_Page_0::GYR_OFFSET_Y_LSB, calibData[8]);
    write8(BNO055_Registers_Page_0::GYR_OFFSET_Y_MSB, calibData[9]);
    write8(BNO055_Registers_Page_0::GYR_OFFSET_Z_LSB, calibData[10]);
    write8(BNO055_Registers_Page_0::GYR_OFFSET_Z_MSB, calibData[11]);

    write8(BNO055_Registers_Page_0::MAG_OFFSET_X_LSB, calibData[12]);
    write8(BNO055_Registers_Page_0::MAG_OFFSET_X_MSB, calibData[13]);
    write8(BNO055_Registers_Page_0::MAG_OFFSET_Y_LSB, calibData[14]);
    write8(BNO055_Registers_Page_0::MAG_OFFSET_Y_MSB, calibData[15]);
    write8(BNO055_Registers_Page_0::MAG_OFFSET_Z_LSB, calibData[16]);
    write8(BNO055_Registers_Page_0::MAG_OFFSET_Z_MSB, calibData[17]);

    write8(BNO055_Registers_Page_0::ACC_RADIUS_LSB, calibData[18]);
    write8(BNO055_Registers_Page_0::ACC_RADIUS_MSB, calibData[19]);

    write8(BNO055_Registers_Page_0::MAG_RADIUS_LSB, calibData[20]);
    write8(BNO055_Registers_Page_0::MAG_RADIUS_MSB, calibData[21]);

    setMode(modeBkp); // return to previous mode
}

/*
* Write sensor offsets from struct
*/
void BNO055::setSensorOffsets(const BNO055_Offsets_t& offsetType) {
    BNO055_Opmode modeBkp = m_mode;
    setMode(BNO055_Opmode::CONFIG); // ensure we are in config mode
    usleep(25000);

    // write sensor offsets from struct
    write8(ACC_OFFSET_X_LSB, (offsetType.accel_offset_x) & 0x0FF);
    write8(ACC_OFFSET_X_MSB, (offsetType.accel_offset_x >> 8) & 0x0FF);
    write8(ACC_OFFSET_Y_LSB, (offsetType.accel_offset_y) & 0x0FF);
    write8(ACC_OFFSET_Y_MSB, (offsetType.accel_offset_y >> 8) & 0x0FF);
    write8(ACC_OFFSET_Z_LSB, (offsetType.accel_offset_z) & 0x0FF);
    write8(ACC_OFFSET_Z_MSB, (offsetType.accel_offset_z >> 8) & 0x0FF);

    write8(GYR_OFFSET_X_LSB, (offsetType.gyro_offset_x) & 0x0FF);
    write8(GYR_OFFSET_X_MSB, (offsetType.gyro_offset_x >> 8) & 0x0FF);
    write8(GYR_OFFSET_Y_LSB, (offsetType.gyro_offset_y) & 0x0FF);
    write8(GYR_OFFSET_Y_MSB, (offsetType.gyro_offset_y >> 8) & 0x0FF);
    write8(GYR_OFFSET_Z_LSB, (offsetType.gyro_offset_z) & 0x0FF);
    write8(GYR_OFFSET_Z_MSB, (offsetType.gyro_offset_z >> 8) & 0x0FF);

    write8(MAG_OFFSET_X_LSB, (offsetType.mag_offset_x) & 0x0FF);
    write8(MAG_OFFSET_X_MSB, (offsetType.mag_offset_x >> 8) & 0x0FF);
    write8(MAG_OFFSET_Y_LSB, (offsetType.mag_offset_y) & 0x0FF);
    write8(MAG_OFFSET_Y_MSB, (offsetType.mag_offset_y >> 8) & 0x0FF);
    write8(MAG_OFFSET_Z_LSB, (offsetType.mag_offset_z) & 0x0FF);
    write8(MAG_OFFSET_Z_MSB, (offsetType.mag_offset_z >> 8) & 0x0FF);

    write8(ACC_RADIUS_LSB, (offsetType.accel_radius) & 0x0FF);
    write8(ACC_RADIUS_MSB, (offsetType.accel_radius >> 8) & 0x0FF);

    write8(MAG_RADIUS_LSB, (offsetType.mag_radius) & 0x0FF);
    write8(MAG_RADIUS_MSB, (offsetType.mag_radius >> 8) & 0x0FF);

    setMode(modeBkp); // return to previous mode
}

/*
* See if the BNO055's sensors are calibrated
*/
bool BNO055::isFullyCalibrated() {
    uint8_t sys, gyro, accel, mag;
    getCalibration(&sys, &gyro, &accel, &mag);
    if(sys < 3 || gyro < 3 || accel < 3 || mag < 3) // if all calibration data == 3 then bno is calibrated
        return false;
    return true;
}

/*
* Write a byte to the i2c_smbus
*/
bool BNO055::write8(uint8_t reg, uint8_t value) {
    return !(i2c_smbus_write_byte_data(m_fdBNO, reg, value) < 0);
}

/*
* Read a byte from register
*/
uint8_t BNO055::read8(uint8_t reg) {
    return static_cast<uint8_t>(i2c_smbus_read_byte_data(m_fdBNO, reg));
}

/*
* Read len number of bytes from register into buffer and return if successful
* Ensure Kernel is > 2.6.22 (implementation of i2c_smbus_read_block changed)
*/
bool BNO055::readLen(uint8_t reg, uint8_t* buf, uint8_t len) {
    // ENSURE KERNEL > 2.6.22
    int bytesRead = i2c_smbus_read_i2c_block_data(m_fdBNO, reg, len, buf);
    if(bytesRead != static_cast<int>(len)) {
        return -1;
    }

    return true;
}