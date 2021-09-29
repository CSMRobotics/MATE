#include "ServoTest.hpp"
#include "ServoDriver.hpp"

void ServoTest::TEST(int iterations) {
    ServoDriver* driver = new ServoDriver();
    driver->addServo(0);

    driver->setAngle(0, 0);
    for(int i=0; i < iterations; i++) {
        driver->setAngle(0, 0);
        sleep(5);
        driver->setAngle(0, 180);
        sleep(5);
    }
}