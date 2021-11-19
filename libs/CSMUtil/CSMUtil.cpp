#include "CSMUtil.hpp"

using namespace csmutil;

PIDController::PIDController (float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

PIDController::PIDController (float kp, float ki, float kd, float maxOutput) {
    PIDController(kp, ki, kd);
    this->maxOutput = maxOutput;
    this->minOutput = -maxOutput;
}

PIDController::PIDController (float kp, float ki, float kd, float maxOutput, float minOutput) {
    PIDController(kp, ki, kd);
    this->maxOutput = maxOutput;
    this->minOutput = minOutput;
}

void PIDController::Update(float error, uint64_t dt) {
    // Compute integral term
    float integral = integralPrior + error*dt;
    float iTerm = integral*ki;

    // Prevents integral windup
    iTerm = iTerm > maxOutput ? maxOutput : iTerm;
    iTerm = iTerm < minOutput ? minOutput : iTerm; 

    // Compute output and clamp it
    float derivative = (error - errorPrior)/dt;
    output = kp*error + iTerm + kd*derivative;
    output = output > maxOutput ? maxOutput : output;
    output = output < minOutput ? minOutput : output;

    // Invert if set to invert
    if (invertOutput) {
        output *= -1;
    }

    errorPrior = error;
    integralPrior = integral;
}

void PIDController::setIntegral(float integralValue) {
    integralPrior = integralValue;
}

void PIDController::invertFeedback(bool invert) {
    invertOutput = invert;
}

float PIDController::getOutput() {
    return output;
}

NonLinearQuaternionController::NonLinearQuaternionController() :
    m_Pq(0), m_Pw(0) {

}

NonLinearQuaternionController::NonLinearQuaternionController(float Pq, float Pw) {
    m_Pq = Pq;
    m_Pw = Pw;
}

Vector3f NonLinearQuaternionController::Update(Quaternionf qref, Quaternionf qm, Vector3f w) {
    Quaternionf qerr = qref * qm.getConjugate();
    Vector3f Axiserr = qerr.getVector();
    Axiserr *= m_Pq;
    w *= m_Pw;
    return Axiserr - w;
}