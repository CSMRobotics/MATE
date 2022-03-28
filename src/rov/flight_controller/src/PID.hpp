#ifndef PID_HPP
#define PID_HPP

#include <functional>
#include <chrono>

class LinearPID {
public:
    LinearPID(float kp = 0.0, float ki = 0.0, float kd = 0.0);

    // Update the controller optional argument dt_us for artificial time
    float Update(long long dt_us = NULL);

    // Set the accumulated integral value. Units are in error*time.
    void setIntegral(float integralValue);
    
    // Invert the direction of the recieved output for PID controllers
    // Producing positive feedback loops instead of negative ones
    void invertFeedback(bool invert);

    // Current output from the PID controller
    float getOutput();

    void setSetpoint(float setpoint);

    void setFeedbackFunction(std::function<float(void)> func);
private:
    float kp, ki, kd;
    float kfb = 1.0;

    float setpoint;
    float integral;
    float output;
    float lastOutput;

    bool doInvertFeedback = false;

    std::function<float(void)> feedback;
    float calcError();
    float calcProportional();
    float calcIntegral();
    float calcDerivative();
};

class NonLinearPI {
public:

private:

};

#endif // PID_HPP