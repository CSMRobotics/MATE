#include "Drive.hpp"

Drive::Drive() {
    timeInitialMillis = millis();
    timeCurrentMillis = timeInitialMillis;
}

Drive::~Drive() {

}

void Drive::Update() {
    timePreviousMillis = timeCurrentMillis;
    timeCurrentMillis = millis();
}

void Drive::AutoUpdate() {

}

void Drive::Stop() {

}