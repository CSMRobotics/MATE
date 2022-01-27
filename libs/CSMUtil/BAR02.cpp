#include "BAR02.hpp"

#include <cmath>
extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

csmutil::MS5837::MS5837() {
    // create fd to i2c file
    // initialize any member variables that need to be
}

void csmutil::MS5837::Update() {

}

void csmutil::MS5837::AutoUpdate() {
    Update();
}

void csmutil::MS5837::Stop() {
    return;
}

float pressure(float conversion=1.0f) {

}

float temperature() {

}

float depth() {

}

float altitude() {

}

void calculate() {

}