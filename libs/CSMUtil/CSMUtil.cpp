#include "CSMUtil.hpp"

using namespace csmutil;

LinearPIDController::LinearPIDController (float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

LinearPIDController::LinearPIDController (float kp, float ki, float kd, float maxOutput) {
    LinearPIDController(kp, ki, kd);
    this->maxOutput = maxOutput;
    this->minOutput = -maxOutput;
}

LinearPIDController::LinearPIDController (float kp, float ki, float kd, float maxOutput, float minOutput) {
    LinearPIDController(kp, ki, kd);
    this->maxOutput = maxOutput;
    this->minOutput = minOutput;
}

void LinearPIDController::Update(float error, uint64_t dt) {
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

void LinearPIDController::setIntegral(float integralValue) {
    integralPrior = integralValue;
}

void LinearPIDController::invertFeedback(bool invert) {
    invertOutput = invert;
}

float LinearPIDController::getOutput() {
    return output;
}

NonLinearQuaternionController::NonLinearQuaternionController() :
    m_Pq(0), m_Pw(0) {

}

NonLinearQuaternionController::NonLinearQuaternionController(float Pq, float Pw) {
    m_Pq = Pq;
    m_Pw = Pw;
}

Vector3d NonLinearQuaternionController::Update(Quaterniond qref, Quaterniond qm, Vector3d w) {
    Quaterniond qerr = qref * qm.getConjugate();
    Vector3d Axiserr = qerr.getVector();
    Axiserr *= m_Pq;
    w *= m_Pw;
    return Axiserr - w;
}