#ifndef THRUSTER_HPP
#define THRUSTER_HPP

#include <algorithm>
#include <array>

#include "Eigen/Dense"
#include "SubConfig.h"

#define DEADZONE 0.1f
#define MIN_THRUST_VALUE -28.44     // N
#define MAX_THRUST_VALUE 36.4       // N
#define MIN_THROTTLE_CUTOFF -0.1
#define MAX_THROTTLE_CUTOFF 0.1
#define MAX_VELOCITY_VALUE 0.0
#define MIN_VELOCITY_VALUE 0.0
#define MAX_OMEGA_VALUE 1.0         // rad/s
#define MIN_OMEGA_VALUE 1.0         // rad/s

struct Thruster {
    Eigen::Vector3f position; // normalized relative positioning of thruster on robot using NED coords
    Eigen::Vector3f thrust; // normalized thrust vector on robot using NED coords
    int64_t controller_idx;
};

inline static std::array<Thruster, NUM_THRUSTERS> load_thrusters()
{
    return {{
        {Eigen::Vector3f FRONT_LEFT_POS, Eigen::Vector3f FRONT_LEFT_THRUST, FRONT_LEFT_IDX},
        {Eigen::Vector3f FRONT_RIGHT_POS, Eigen::Vector3f FRONT_RIGHT_THRUST, FRONT_RIGHT_IDX},
        {Eigen::Vector3f BACK_LEFT_POS, Eigen::Vector3f BACK_LEFT_THRUST, BACK_LEFT_IDX},
        {Eigen::Vector3f BACK_RIGHT_POS, Eigen::Vector3f BACK_RIGHT_THRUST, BACK_RIGHT_IDX},

        {Eigen::Vector3f MID_FRONT_LEFT_POS, Eigen::Vector3f MID_FRONT_LEFT_THRUST, MID_FRONT_LEFT_IDX},
        {Eigen::Vector3f MID_FRONT_RIGHT_POS, Eigen::Vector3f MID_FRONT_RIGHT_THRUST, MID_FRONT_RIGHT_IDX},
        {Eigen::Vector3f MID_BACK_LEFT_POS, Eigen::Vector3f MID_BACK_LEFT_THRUST, MID_BACK_LEFT_IDX},
        {Eigen::Vector3f MID_BACK_RIGHT_POS, Eigen::Vector3f MID_BACK_RIGHT_THRUST, MID_BACK_RIGHT_IDX}
    }};
}

inline static float thrust_from_throttle(const float throttle) {
    assert(abs(throttle) <= 1);
    // check if throttle is greater than deadzone (symmetric)
    if(abs(throttle) >= DEADZONE) {
        // faster than floating point operation :)
        // WTF Explanation: This checks the sign bit but more fastly than a comparison to zero.
        if(*reinterpret_cast<const int32_t*>(&throttle) & 0x80000000) {
            // positive thrust curve derived from BlueRobotics' testing
            return -1.23 + 15.9 * throttle + 23.2 * throttle * throttle;
        } else {
            // negative thrust curve derived from BlueRobotics' testing
            return 1.05 + 13.5 * throttle - 17.1 * throttle * throttle;
        }
    }
    return 0;
}

inline static float throttle_from_thrust(const float thrust, bool clamping = true) {
    switch(clamping) {
        case true:
            // inverse thrust function
            if(thrust < 0) {
                // negative throttle curve deried from BlueRobotics' testing
                return std::max<float>(-0.0991 + 0.0505 * thrust + 1.22e-3 * thrust * thrust + 1.91e-5 * thrust * thrust * thrust,-1.0);
            } else if (thrust > 0) {
                // positive throttle curve deried from BlueRobotics' testing
                return std::min<float>(0.0986 + 0.0408 * thrust + -8.14e-4 * thrust * thrust + 1.01e-5 * thrust * thrust * thrust,1.0);
            } else {
                return 0;
            }
        break;
        case false:
            if(thrust < 0) {
                // negative throttle curve deried from BlueRobotics' testing
                return -0.0991 + 0.0505 * thrust + 1.22e-3 * thrust * thrust + 1.91e-5 * thrust * thrust * thrust;
            } else if (thrust > 0) {
                // positive throttle curve deried from BlueRobotics' testing
                return 0.0986 + 0.0408 * thrust + -8.14e-4 * thrust * thrust + 1.01e-5 * thrust * thrust * thrust;
            } else {
                return 0;
            }
        break;
    }
}

#endif //THRUSTER_HPP
