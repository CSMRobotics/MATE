#include "../include/IMU.hpp>"

IMU::IMU() {
    //TODO: we need to supply the bno055 struct with a delay function
    struct bno055_t bno055;
    s32 comres = BNO055_ERROR;
    comres = bno055_init(&bno055);

    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
 
    double accel_datax = BNO055_INIT_VALUE;

    double accel_datay = BNO055_INIT_VALUE;

    double accel_dataz = BNO055_INIT_VALUE;
    comres += bno055_convert_double_accel_x_msq(&accel_datax);
    comres += bno055_convert_double_accel_y_msq(&accel_datay);
    comres += bno055_convert_double_accel_z_msq(&accel_dataz);
    std::cout << "x accel: " << accel_datax << std::endl;
    std::cout << "y accel: " << accel_datay << std::endl;
    std::cout << "z accel: " << accel_dataz << std::endl;
}

IMU::~IMU() {

}

void IMU::Update() {

}

void IMU::AutoUpdate() {

}

void IMU::Stop() {

}