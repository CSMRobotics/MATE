#ifndef SUB_CONFIG_H
#define SUB_CONFIG_H

/*********************************************************
 * PID Config
 ********************************************************/
#define MAX_OMEGA 0.78539816f     // rad/s
#define MAX_VERTICAL_VELOCITY 2.0f   // m/s

#define ORIENT_KP 1.0f
#define ORIENT_KI 0.5f
#define ORIENT_KD 0.25f

#define ALTITUDE_KP 1.0f
#define ALTITUDE_KI 0.5f
#define ALTITUDE_KD 0.25f

/*********************************************************
 * Thrusters
 ********************************************************/
#define NUM_THRUSTERS 8

#define FRONT_LEFT_POS {1.0f, -0.5f, 0.0f}  // Normalized coordinates in NED
#define FRONT_LEFT_THRUST {0.707106781f, 0.707106781f, 0.0f}   // Normalized thrust vector in NED
#define FRONT_LEFT_IDX 0    // ThrottleController index

#define FRONT_RIGHT_POS {1.0f, 0.5f, 0.0f}
#define FRONT_RIGHT_THRUST {0.707106781f, -0.707106781f, 0.0f}
#define FRONT_RIGHT_IDX 1

#define BACK_LEFT_POS {-1.0f, -0.5f, 0.0f}
#define BACK_LEFT_THRUST {-0.707106781f, 0.707106781f, 0.0f}
#define BACK_LEFT_IDX 2

#define BACK_RIGHT_POS {-1.0f, 0.5f, 0.0f}
#define BACK_RIGHT_THRUST {-0.707106781f, -0.707106781f, 0.0f}
#define BACK_RIGHT_IDX 3

#define MID_FRONT_LEFT_POS {0.75f, -0.5f, 0.0f}
#define MID_FRONT_LEFT_THRUST {0.0f, 0.0f, 1.0f}
#define MID_FRONT_LEFT_IDX 4

#define MID_FRONT_RIGHT_POS {0.75f, 0.5f, 0.0f}
#define MID_FRONT_RIGHT_THRUST {0.0f, 0.0f, 1.0f}
#define MID_FRONT_RIGHT_IDX 5

#define MID_BACK_LEFT_POS {-0.75f, -0.5f, 0.0f}
#define MID_BACK_LEFT_THRUST {0.0f, 0.0f, 1.0f}
#define MID_BACK_LEFT_IDX 6

#define MID_BACK_RIGHT_POS {-0.75f, 0.5f, 0.0f}
#define MID_BACK_RIGHT_THRUST {0.0f, 0.0f, 1.0f}
#define MID_BACK_RIGHT_IDX 7

#endif