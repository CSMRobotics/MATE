#ifndef CSMUTIL_HPP
#define CSMUTIL_HPP

#include <cmath>
#include <iostream>
#include <algorithm>

#include "Vector3.hpp"
#include "Quaternion.hpp"

namespace csmutil{

static int imap(float value, float from_min, float from_max, int to_min, int to_max){
    return ((value - from_min) * ((to_max - to_min) / (from_max - from_min)) + to_min);
}

template<typename R, typename T, typename F>
static R map(F value, F from_min, F from_max, T to_min, T to_max) {
    return (R)((value - from_min) * (to_max - to_min) / (from_max - from_min) + to_min);
}

class LinearPIDController {
public:
    LinearPIDController() = default;
    LinearPIDController(float kp, float ki, float kd);
    LinearPIDController(float kp, float ki, float kd, float maxOutput);
    LinearPIDController(float kp, float ki, float kd, float maxOutput, float minOutput);

    void Update(float error, uint64_t dt);

    // Set the accumulated integral value. Units are in error*time.
    void setIntegral(float integralValue);
    
    // Invert the direction of the recieved output for PID controllers
    // Producing positive feedback loops instead of negative ones
    void invertFeedback(bool invert);

    // Current output from the PID controller
    float getOutput();
private:
    // Gains
    float kp;
    float ki;
    float kd;

    // Bounds
    float maxOutput = 1;
    float minOutput = -1;

    // Saved values
    float errorPrior = 0;
    float integralPrior = 0;
    float output = 0;

    // Whether to invert the direction of the output
    bool invertOutput = false;
};


class NonLinearQuaternionController {
public:
    NonLinearQuaternionController();
    NonLinearQuaternionController(float Pq, float Pw);

    void setPq(float Pq) {this->m_Pq = Pq;};
    void setPw(float Pw) {this->m_Pw = Pw;};

    float getPq() {return this->m_Pq;};
    float getPw() {return this->m_Pw;};

    Vector3d Update(Quaterniond qref, Quaterniond qm, Vector3d w);
private:
    float m_Pq, m_Pw;
};

};

#endif // CSMUTIL_HPP