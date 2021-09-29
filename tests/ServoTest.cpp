#include "ServoTest.hpp"
#include "ServoDriver.hpp"

void TEST() {
    ServoDriver* driver = new ServoDriver();
    driver->addServo(0);

    driver->setAngle(0, 0);
    while(true) {
        driver->setAngle(0, 0);
        sleep(5);
        driver->setAngle(0, 180);
        sleep(5);
    }
}