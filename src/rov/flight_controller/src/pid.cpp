#include "flight_controller/pid.hpp"

#include <algorithm>

PID::PID(bool P_enabled=true, bool I_enabled=true, bool D_enabled=true) {
    p_en = P_enabled;
    i_en = I_enabled;
    d_en = D_enabled;
}

float PID::update(float dt, float measurement, float setpoint) {
    float error = setpoint-measurement;
    float p = p_en ? P(error) : 0.0f;
    float i = i_en ? I(dt, error) : 0.0f;
    float d = d_en ? D(dt, error) : 0.0f;
    float last_output = p+i+d;
    return last_output;
}

void PID::setWindupMax(float maximum_integral_value) {
    max_i = maximum_integral_value;
}

void PID::setWindupMin(float minimum_integral_value) {
    min_i = minimum_integral_value;
}

void PID::setWindupMaxMin(float maximum_abs_integral_value) {
    max_i = min_i = maximum_abs_integral_value;
}

void PID::setKP(float kp) {
    this->kp = kp;
}

void PID::setKI(float ki) {
    this->ki = ki;
}

void PID::setKD(float kd) {
    this->kd = kd;
}

void PID::setTi(float Ti) {
    ki = kp/Ti;
}

void PID::setTi(float Ti, float kp) {
    this->kp = kp;
    ki = kp/Ti;
}

void PID::setTd(float Td) {
    kd = kp/Td;
}

void PID::setTd(float Td, float kp) {
    this->kp = kp;
    kd = kp/Td;
}


// Kp * e(t)
float PID::P(float error) {
    return kp*error;
}

// Ki * integral 0->t [e(tau) dtau]
float PID::I(float dt, float error) {
    static float integral = 0;
    integral += error*dt;
    integral = std::max(integral, min_i);
    integral = std::min(integral, max_i);

    return integral;
}

// Kd * de(t)/dt
float PID::D(float dt, float error) {
    static float e_last = 0;
    float val = (e_last - error)/dt;
    e_last = error;
    return val;
}