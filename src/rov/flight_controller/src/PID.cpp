#include "PID.hpp"

LinearPID::LinearPID(float kp, float ki, float kd, float kfb) {
    this->initTime = std::chrono::high_resolution_clock::now();
    this->timelast = this->initTime;

    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->kfb = kfb;
}

float LinearPID::Update(long long dt_us) {
    if(dt_us == NULL) { // if no supplied time, use chrono clock :)
        if (this->timelast == this->initTime) {
            this->dt_us = 10; // default to 10 micro seconds since last update for first update
            this->error = 0; // default to 0 error
        } else {
            this->dt_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - this->timelast).count();
        }
    } else {
        this->dt_us = dt_us;
    }
    this->lastOutput = this->output;
    this->lastError = this->error;
    this->error = this->calcError();
    this->integral = this->calcIntegral();
    this->output = this->integral + this->calcProportional() + this->calcDerivative() + bias;
}

float LinearPID::calcError() {
    this->setpoint + ((this->doInvertFeedback - 1) * 2 + 1) * feedback();
}

float LinearPID::calcProportional() {
    return this->error * this->kp;
}

float LinearPID::calcIntegral() {
    return this->integral + this->error * this->dt_us;
}

float LinearPID::calcDerivative() {
    return (this->error - this->lastError) / dt_us;
}