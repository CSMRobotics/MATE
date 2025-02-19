#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID(bool P_enabled=true, bool I_enabled=true, bool D_enabled=true);

    float update(float dt, float measurement, float setpoint);

    void setWindupMax(float maximum_integral_value);
    void setWindupMin(float minimum_integral_value);
    void setWindupMaxMin(float maximum_abs_integral_value);

    void setKP(float kp);
    void setKI(float ki);
    void setKD(float kd);
    
    void setTi(float Ti);
    void setTi(float Ti, float kp);
    void setTd(float Td);
    void setTd(float Td, float kp);
private:
    float P(float error);
    float I(float dt, float error);
    float D(float dt, float error);

    float kp, ki, kd;
    float max_i;
    float min_i;
    bool p_en, i_en, d_en;
    float last_output;
};

#endif