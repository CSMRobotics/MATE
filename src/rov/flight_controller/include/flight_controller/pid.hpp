#ifndef PID_HPP
#define PID_HPP

#include <algorithm>
#include <eigen3/Eigen/Eigen>

struct ClassAwareLexicographicLess {
    template<class T>
    constexpr bool operator()(const T& a, const T& b) {
        if constexpr (!std::is_class<T>::value) {
            return a < b;
        } else {
            return std::lexicographical_compare(a.begin(), a.end(), b.begin(), b.end());
        }
    }
};

template<typename T>
class PID {
public:
    PID(bool P_enabled=true, bool I_enabled=true, bool D_enabled=true) {
        p_en = P_enabled;
        i_en = I_enabled;
        d_en = D_enabled;
        max_i = T{};
        min_i = T{};
        kp = ki = kd = 1.0f;
        last_output = T{};
    };

    T update(float dt, T measurement, T setpoint) {
        T error = setpoint-measurement;
        T p = p_en ? P(error) : T{};
        T i = i_en ? I(dt, error) : T{};
        T d = d_en ? D(dt, error) : T{};
        last_output = p+i+d;
        return last_output;
    }

    void setWindupMax(T maximum_integral_value) {
        max_i = maximum_integral_value;
    };

    void setWindupMin(T minimum_integral_value) {
        min_i = minimum_integral_value;
    };

    void setWindupMaxMin(T maximum_abs_integral_value) {
        max_i = min_i = maximum_abs_integral_value;
    };

    void setKP(float kp) {
        this->kp = kp;
    };
    void setKI(float ki) {
        this->ki = ki;
    };
    void setKD(float kd) {
        this->kd = kd;
    };
    
    void setTi(float Ti) {
        ki = kp/Ti;
    };
    void setTi(float Ti, float kp) {
        this->kp = kp;
        ki = kp/Ti;
    };
    void setTd(float Td) {
        kd = kp/Td;
    };
    void setTd(float Td, float kp) {
        this->kp = kp;
        kd = kp/Td;
    };
private:
    T P(T error) {return kp*error;};
    T I(float dt, T error) {
        static T integral = T{};
        integral += dt*error;
        integral = std::max(integral, min_i, ClassAwareLexicographicLess{});
        integral = std::min(integral, max_i, ClassAwareLexicographicLess{});
    
        return integral;
    };
    T D(float dt, T error) {
        static T e_last = T{};
        T val = (e_last - error)/dt;
        e_last = error;
        return val;
    };

    float kp, ki, kd;
    T max_i;
    T min_i;
    bool p_en, i_en, d_en;
    T last_output;
};

#endif