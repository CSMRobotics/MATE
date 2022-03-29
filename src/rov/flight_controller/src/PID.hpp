#ifndef PID_HPP
#define PID_HPP

#include <functional>
#include <chrono>

class LinearPID {
public:
    LinearPID(float kp = 0.0, float ki = 0.0, float kd = 0.0, float kfb = 1.0);

    // Update the controller optional argument dt_us for artificial time
    float Update(long long dt_us = 0);

    // Set the accumulated integral value. Units are in error*time.
    void setIntegral(float integralValue) {this->integral = integralValue;};
    
    // Invert the direction of the recieved output for PID controllers
    // Producing positive feedback loops instead of negative ones
    void invertFeedback(bool invert) {this->doInvertFeedback = invert;};

    // Current output from the PID controller
    float getOutput() {return this->output;};

    void setSetpoint(float setpoint) {this->setpoint = setpoint;};

    void setFeedbackFunction(std::function<float(void)> func) {this->feedback = func;};

    void setBias(float bias) {this->bias = bias;};
private:
    // PID gain constant
    float kp, ki, kd;
    // feedback gain constant
    float kfb;

    long long dt_us;

    float setpoint;
    float error;
    float lastError;
    float integral;
    float output = 0;
    float lastOutput = output;
    float bias = 0.0;

    std::chrono::time_point<std::chrono::high_resolution_clock> initTime;
    std::chrono::time_point<std::chrono::high_resolution_clock> timelast;

    bool doInvertFeedback = false;

    std::function<float(void)> feedback = std::bind(&LinearPID::defaultFeedback, this);
    float calcError();
    float calcProportional();
    float calcIntegral();
    float calcDerivative();

    float defaultFeedback() {return 0.0;};
};

class NonLinearPI {
public:

private:

};

#endif // PID_HPP