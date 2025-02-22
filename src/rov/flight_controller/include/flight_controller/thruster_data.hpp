#ifndef THRUSTER_HPP
#define THRUSTER_HPP

#include <cmath>
#include <cstdint>

// EMPIRICAL FIT TO PUBLISHED T200 12V Data
#define NEGATIVE_BEST_FIT_A -9.16411681345042E-5
#define NEGATIVE_BEST_FIT_B 0.316235205225523
#define NEGATIVE_BEST_FIT_C -266.314177431551
#define NEGATIVE_BEST_FIT_MIN_PWM 1100
#define NEGATIVE_BEST_FIT_MAX_PWM 1460

#define NEGATIVE_BEST_FIT_INV_A 1.63923346235904E-1
#define NEGATIVE_BEST_FIT_INV_B 16.6548605581493
#define NEGATIVE_BEST_FIT_INV_C 1452.75018853837
#define NEGATIVE_BEST_FIT_INV_MIN_THRUST -24.48
#define NEGATIVE_BEST_FIT_INV_MAX_THRUST -0.5


#define DEADZONE_MIN_PWM 1461
#define DEADZONE_MAX_PWM 1539


#define POSITIVE_BEST_FIT_A 1.27100189603788E-4
#define POSITIVE_BEST_FIT_B -0.33277857760984
#define POSITIVE_BEST_FIT_C 211.033461129255
#define POSITIVE_BEST_FIT_MIN_PWM 1540
#define POSITIVE_BEST_FIT_MAX_PWM 1900

#define POSITIVE_BEST_FIT_INV_A -1.0868853558235E-1
#define POSITIVE_BEST_FIT_INV_B 13.2936060632298
#define POSITIVE_BEST_FIT_INV_C 1547.69996899387
#define POSITIVE_BEST_FIT_INV_MIN_THRUST 0.5
#define POSITIVE_BEST_FIT_INV_MAX_THRUST 36.42



#define FULL_REVERSE_THRUST_PWM 1100
#define ZERO_THRUST_PWM 1500
#define FULL_FORWARD_THRUST_PWM 1900

namespace {
    float t200_test_data_positive_pwm_to_thrust(const float pwm) {
        return POSITIVE_BEST_FIT_A * pwm * pwm + POSITIVE_BEST_FIT_B * pwm + POSITIVE_BEST_FIT_C;
    }

    float t200_test_data_negative_pwm_to_thrust(const float pwm) {
        return NEGATIVE_BEST_FIT_A * pwm * pwm + NEGATIVE_BEST_FIT_B * pwm + NEGATIVE_BEST_FIT_C;
    }

    float t200_test_data_positive_thrust_to_pwm(const float thrust) {
        return POSITIVE_BEST_FIT_INV_A * thrust * thrust + POSITIVE_BEST_FIT_INV_B * thrust  + POSITIVE_BEST_FIT_INV_C;
    }

    float t200_test_data_negative_thrust_to_pwm(const float thrust) {
        return NEGATIVE_BEST_FIT_INV_A * thrust * thrust + NEGATIVE_BEST_FIT_INV_B * thrust  + NEGATIVE_BEST_FIT_INV_C;
    }
}

inline static float thrust_from_pwm(const float pwm) {
    // always round towards 0-thrust
    // are we in the positive or negative regions?
    if (pwm > POSITIVE_BEST_FIT_MIN_PWM) {
        float res = std::floorf(t200_test_data_positive_pwm_to_thrust(pwm));
        return res > POSITIVE_BEST_FIT_INV_MAX_THRUST ? FULL_FORWARD_THRUST_PWM : res;
    } else if (pwm < NEGATIVE_BEST_FIT_MAX_PWM) {
        float res = std::ceilf(t200_test_data_negative_pwm_to_thrust(pwm));
        return res < NEGATIVE_BEST_FIT_INV_MIN_THRUST ? NEGATIVE_BEST_FIT_INV_MIN_THRUST : res;
    }
    // else we are in the deadzone
    return 0.0f;
}

inline static uint16_t pwm_from_thrust(const float thrust, bool clamping = true) {
    // always round towards 0-thrust pwm
    // are we in the positive or negative regions?
    if (thrust > POSITIVE_BEST_FIT_INV_MIN_THRUST) {
        uint16_t res = static_cast<uint16_t>(std::floorf(t200_test_data_positive_thrust_to_pwm(thrust)));
        return clamping ? (res > FULL_FORWARD_THRUST_PWM ? FULL_FORWARD_THRUST_PWM : res) : res;
    } else if (thrust < NEGATIVE_BEST_FIT_INV_MAX_THRUST) {
        uint16_t res = static_cast<uint16_t>(std::ceilf(t200_test_data_negative_thrust_to_pwm(thrust)));
        return clamping ? (res < FULL_REVERSE_THRUST_PWM ? FULL_REVERSE_THRUST_PWM : res) : res;
    }
    // else we are in the deadzone
    return ZERO_THRUST_PWM;
}

#endif //THRUSTER_HPP
