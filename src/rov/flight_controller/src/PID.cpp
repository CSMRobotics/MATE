#include "PID.hpp"


LinearPID::LinearPID(float kp, float ki, float kd) {

}

float LinearPID::Update(long long dt_us) {
    if(dt_us == NULL) {

    }
}

void LinearPID::setIntegral(float integralValue) {
    this->integral = integralValue;
}

void LinearPID::invertFeedback(bool invert) {
    this->doInvertFeedback = invert;   
}

float LinearPID::getOutput() {
    return output;
}

void LinearPID::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
}

void LinearPID::setFeedbackFunction(std::function<float(void)> func) {
    this->feedback = func;
}

