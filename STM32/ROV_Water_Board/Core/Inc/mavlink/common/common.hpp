/** @file
 *  @brief MAVLink comm protocol generated from common.xml
 *  @see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include <mavlink/message.hpp>

namespace mavlink {
namespace common {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 235> MESSAGE_ENTRIES {{ {0, 50, 9, 9, 0, 0, 0}, {1, 124, 31, 43, 0, 0, 0}, {2, 137, 12, 12, 0, 0, 0}, {4, 237, 14, 14, 3, 12, 13}, {5, 217, 28, 28, 1, 0, 0}, {6, 104, 3, 3, 0, 0, 0}, {7, 119, 32, 32, 0, 0, 0}, {8, 117, 36, 36, 0, 0, 0}, {11, 89, 6, 6, 1, 4, 0}, {20, 214, 20, 20, 3, 2, 3}, {21, 159, 2, 2, 3, 0, 1}, {22, 220, 25, 25, 0, 0, 0}, {23, 168, 23, 23, 3, 4, 5}, {24, 24, 30, 52, 0, 0, 0}, {25, 23, 101, 101, 0, 0, 0}, {26, 170, 22, 24, 0, 0, 0}, {27, 144, 26, 29, 0, 0, 0}, {28, 67, 16, 16, 0, 0, 0}, {29, 115, 14, 16, 0, 0, 0}, {30, 39, 28, 28, 0, 0, 0}, {31, 246, 32, 48, 0, 0, 0}, {32, 185, 28, 28, 0, 0, 0}, {33, 104, 28, 28, 0, 0, 0}, {34, 237, 22, 22, 0, 0, 0}, {35, 244, 22, 22, 0, 0, 0}, {36, 222, 21, 37, 0, 0, 0}, {37, 212, 6, 7, 3, 4, 5}, {38, 9, 6, 7, 3, 4, 5}, {39, 254, 37, 38, 3, 32, 33}, {40, 230, 4, 5, 3, 2, 3}, {41, 28, 4, 4, 3, 2, 3}, {42, 28, 2, 18, 0, 0, 0}, {43, 132, 2, 3, 3, 0, 1}, {44, 221, 4, 9, 3, 2, 3}, {45, 232, 2, 3, 3, 0, 1}, {46, 11, 2, 2, 0, 0, 0}, {47, 153, 3, 8, 3, 0, 1}, {48, 41, 13, 21, 1, 12, 0}, {49, 39, 12, 20, 0, 0, 0}, {50, 78, 37, 37, 3, 18, 19}, {51, 196, 4, 5, 3, 2, 3}, {54, 15, 27, 27, 3, 24, 25}, {55, 3, 25, 25, 0, 0, 0}, {61, 167, 72, 72, 0, 0, 0}, {62, 183, 26, 26, 0, 0, 0}, {63, 119, 181, 181, 0, 0, 0}, {64, 191, 225, 225, 0, 0, 0}, {65, 118, 42, 42, 0, 0, 0}, {66, 148, 6, 6, 3, 2, 3}, {67, 21, 4, 4, 0, 0, 0}, {69, 243, 11, 30, 1, 10, 0}, {70, 124, 18, 38, 3, 16, 17}, {73, 38, 37, 38, 3, 32, 33}, {74, 20, 20, 20, 0, 0, 0}, {75, 158, 35, 35, 3, 30, 31}, {76, 152, 33, 33, 3, 30, 31}, {77, 143, 3, 10, 3, 8, 9}, {80, 14, 4, 4, 3, 2, 3}, {81, 106, 22, 22, 0, 0, 0}, {82, 49, 39, 51, 3, 36, 37}, {83, 22, 37, 37, 0, 0, 0}, {84, 143, 53, 53, 3, 50, 51}, {85, 140, 51, 51, 0, 0, 0}, {86, 5, 53, 53, 3, 50, 51}, {87, 150, 51, 51, 0, 0, 0}, {89, 231, 28, 28, 0, 0, 0}, {90, 183, 56, 56, 0, 0, 0}, {91, 63, 42, 42, 0, 0, 0}, {92, 54, 33, 33, 0, 0, 0}, {93, 47, 81, 81, 0, 0, 0}, {100, 175, 26, 34, 0, 0, 0}, {101, 102, 32, 117, 0, 0, 0}, {102, 158, 32, 117, 0, 0, 0}, {103, 208, 20, 57, 0, 0, 0}, {104, 56, 32, 116, 0, 0, 0}, {105, 93, 62, 63, 0, 0, 0}, {106, 138, 44, 44, 0, 0, 0}, {107, 108, 64, 65, 0, 0, 0}, {108, 32, 84, 92, 0, 0, 0}, {109, 185, 9, 9, 0, 0, 0}, {110, 84, 254, 254, 3, 1, 2}, {111, 34, 16, 18, 3, 16, 17}, {112, 174, 12, 12, 0, 0, 0}, {113, 124, 36, 39, 0, 0, 0}, {114, 237, 44, 44, 0, 0, 0}, {115, 4, 64, 64, 0, 0, 0}, {116, 76, 22, 24, 0, 0, 0}, {117, 128, 6, 6, 3, 4, 5}, {118, 56, 14, 14, 0, 0, 0}, {119, 116, 12, 12, 3, 10, 11}, {120, 134, 97, 97, 0, 0, 0}, {121, 237, 2, 2, 3, 0, 1}, {122, 203, 2, 2, 3, 0, 1}, {123, 250, 113, 113, 3, 0, 1}, {124, 87, 35, 57, 0, 0, 0}, {125, 203, 6, 6, 0, 0, 0}, {126, 220, 79, 81, 3, 79, 80}, {127, 25, 35, 35, 0, 0, 0}, {128, 226, 35, 35, 0, 0, 0}, {129, 46, 22, 24, 0, 0, 0}, {130, 29, 13, 13, 0, 0, 0}, {131, 223, 255, 255, 0, 0, 0}, {132, 85, 14, 39, 0, 0, 0}, {133, 6, 18, 18, 0, 0, 0}, {134, 229, 43, 43, 0, 0, 0}, {135, 203, 8, 8, 0, 0, 0}, {136, 1, 22, 22, 0, 0, 0}, {137, 195, 14, 16, 0, 0, 0}, {138, 109, 36, 120, 0, 0, 0}, {139, 168, 43, 43, 3, 41, 42}, {140, 181, 41, 41, 0, 0, 0}, {141, 47, 32, 32, 0, 0, 0}, {142, 72, 243, 243, 0, 0, 0}, {143, 131, 14, 16, 0, 0, 0}, {144, 127, 93, 93, 0, 0, 0}, {146, 103, 100, 100, 0, 0, 0}, {147, 154, 36, 54, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0}, {149, 200, 30, 60, 0, 0, 0}, {162, 189, 8, 9, 0, 0, 0}, {192, 36, 44, 54, 0, 0, 0}, {225, 208, 65, 73, 0, 0, 0}, {230, 163, 42, 42, 0, 0, 0}, {231, 105, 40, 40, 0, 0, 0}, {232, 151, 63, 65, 0, 0, 0}, {233, 35, 182, 182, 0, 0, 0}, {234, 150, 40, 40, 0, 0, 0}, {235, 179, 42, 42, 0, 0, 0}, {241, 90, 32, 32, 0, 0, 0}, {242, 104, 52, 60, 0, 0, 0}, {243, 85, 53, 61, 1, 52, 0}, {244, 95, 6, 6, 0, 0, 0}, {245, 130, 2, 2, 0, 0, 0}, {246, 184, 38, 38, 0, 0, 0}, {247, 81, 19, 19, 0, 0, 0}, {248, 8, 254, 254, 3, 3, 4}, {249, 204, 36, 36, 0, 0, 0}, {250, 49, 30, 30, 0, 0, 0}, {251, 170, 18, 18, 0, 0, 0}, {252, 44, 18, 18, 0, 0, 0}, {253, 83, 51, 54, 0, 0, 0}, {254, 46, 9, 9, 0, 0, 0}, {256, 71, 42, 42, 3, 8, 9}, {257, 131, 9, 9, 0, 0, 0}, {258, 187, 32, 232, 3, 0, 1}, {259, 92, 235, 237, 0, 0, 0}, {260, 146, 5, 14, 0, 0, 0}, {261, 179, 27, 61, 0, 0, 0}, {262, 12, 18, 23, 0, 0, 0}, {263, 133, 255, 255, 0, 0, 0}, {264, 49, 28, 32, 0, 0, 0}, {265, 26, 16, 20, 0, 0, 0}, {266, 193, 255, 255, 3, 2, 3}, {267, 35, 255, 255, 3, 2, 3}, {268, 14, 4, 4, 3, 2, 3}, {269, 109, 213, 215, 0, 0, 0}, {270, 59, 19, 20, 0, 0, 0}, {271, 22, 52, 53, 0, 0, 0}, {275, 126, 31, 32, 0, 0, 0}, {276, 18, 49, 50, 0, 0, 0}, {277, 62, 30, 30, 0, 0, 0}, {280, 70, 33, 33, 0, 0, 0}, {281, 48, 13, 13, 0, 0, 0}, {282, 123, 35, 35, 3, 32, 33}, {283, 74, 144, 149, 0, 0, 0}, {284, 99, 32, 32, 3, 30, 31}, {285, 137, 40, 49, 3, 38, 39}, {286, 210, 53, 57, 3, 50, 51}, {287, 1, 23, 23, 3, 20, 21}, {288, 20, 23, 23, 3, 20, 21}, {290, 251, 46, 46, 0, 0, 0}, {291, 10, 57, 57, 0, 0, 0}, {295, 234, 12, 12, 0, 0, 0}, {296, 158, 41, 41, 3, 36, 37}, {299, 19, 96, 98, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0}, {301, 243, 58, 58, 0, 0, 0}, {310, 28, 17, 17, 0, 0, 0}, {311, 95, 116, 116, 0, 0, 0}, {320, 243, 20, 20, 3, 2, 3}, {321, 88, 2, 2, 3, 0, 1}, {322, 243, 149, 149, 0, 0, 0}, {323, 78, 147, 147, 3, 0, 1}, {324, 132, 146, 146, 0, 0, 0}, {330, 23, 158, 167, 0, 0, 0}, {331, 91, 230, 233, 0, 0, 0}, {332, 236, 239, 239, 0, 0, 0}, {333, 231, 109, 109, 0, 0, 0}, {334, 72, 10, 53, 0, 0, 0}, {335, 225, 24, 24, 0, 0, 0}, {336, 245, 84, 84, 0, 0, 0}, {339, 199, 5, 5, 0, 0, 0}, {340, 99, 70, 70, 0, 0, 0}, {345, 209, 21, 21, 3, 2, 3}, {350, 232, 20, 252, 0, 0, 0}, {360, 11, 25, 25, 0, 0, 0}, {361, 93, 33, 33, 0, 0, 0}, {370, 75, 87, 109, 0, 0, 0}, {371, 10, 26, 26, 0, 0, 0}, {372, 26, 140, 140, 0, 0, 0}, {373, 117, 42, 42, 0, 0, 0}, {375, 251, 140, 140, 0, 0, 0}, {376, 199, 8, 8, 0, 0, 0}, {380, 232, 20, 20, 0, 0, 0}, {385, 147, 133, 133, 3, 2, 3}, {386, 132, 16, 16, 3, 4, 5}, {387, 4, 72, 72, 3, 4, 5}, {388, 8, 37, 37, 3, 32, 33}, {390, 156, 238, 240, 0, 0, 0}, {395, 0, 212, 212, 0, 0, 0}, {396, 50, 160, 160, 0, 0, 0}, {397, 182, 108, 108, 0, 0, 0}, {400, 110, 254, 254, 3, 4, 5}, {401, 183, 6, 6, 3, 4, 5}, {410, 160, 53, 53, 0, 0, 0}, {411, 106, 3, 3, 0, 0, 0}, {412, 33, 6, 6, 3, 4, 5}, {413, 77, 7, 7, 3, 4, 5}, {420, 20, 9, 73, 3, 6, 7}, {435, 134, 46, 47, 0, 0, 0}, {436, 193, 9, 9, 0, 0, 0}, {437, 30, 1, 1, 0, 0, 0}, {440, 66, 35, 35, 0, 0, 0}, {9000, 113, 137, 137, 0, 0, 0}, {9005, 117, 34, 34, 0, 0, 0}, {12900, 114, 44, 44, 3, 0, 1}, {12901, 254, 59, 59, 3, 30, 31}, {12902, 140, 53, 53, 3, 4, 5}, {12903, 249, 46, 46, 3, 0, 1}, {12904, 77, 54, 54, 3, 28, 29}, {12905, 49, 43, 43, 3, 0, 1}, {12915, 94, 249, 249, 3, 0, 1}, {12918, 139, 51, 51, 0, 0, 0}, {12919, 7, 18, 18, 3, 16, 17}, {12920, 20, 5, 5, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS


/** @brief Flags to report failure cases over the high latency telemetry. */
enum class HL_FAILURE_FLAG : uint16_t
{
    GPS=1, /* GPS failure. | */
    DIFFERENTIAL_PRESSURE=2, /* Differential pressure sensor failure. | */
    ABSOLUTE_PRESSURE=4, /* Absolute pressure sensor failure. | */
    FLAG_3D_ACCEL=8, /* Accelerometer sensor failure. | */
    FLAG_3D_GYRO=16, /* Gyroscope sensor failure. | */
    FLAG_3D_MAG=32, /* Magnetometer sensor failure. | */
    TERRAIN=64, /* Terrain subsystem failure. | */
    BATTERY=128, /* Battery failure/critical low battery. | */
    RC_RECEIVER=256, /* RC receiver failure/no RC connection. | */
    OFFBOARD_LINK=512, /* Offboard link failure. | */
    ENGINE=1024, /* Engine failure. | */
    GEOFENCE=2048, /* Geofence violation. | */
    ESTIMATOR=4096, /* Estimator failure, for example measurement rejection or large variances. | */
    MISSION=8192, /* Mission failure. | */
};

//! HL_FAILURE_FLAG ENUM_END
constexpr auto HL_FAILURE_FLAG_ENUM_END = 8193;

/** @brief Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution. */
enum class MAV_GOTO
{
    DO_HOLD=0, /* Hold at the current position. | */
    DO_CONTINUE=1, /* Continue with the next item in mission execution. | */
    HOLD_AT_CURRENT_POSITION=2, /* Hold at the current position of the system | */
    HOLD_AT_SPECIFIED_POSITION=3, /* Hold at the position specified in the parameters of the DO_HOLD action | */
};

//! MAV_GOTO ENUM_END
constexpr auto MAV_GOTO_ENUM_END = 4;

/** @brief Predefined OR-combined MAV_MODE_FLAG values. These can simplify using the flags when setting modes. Note that manual input is enabled in all modes as a safety override. */
enum class MAV_MODE : uint8_t
{
    PREFLIGHT=0, /* System is not ready to fly, booting, calibrating, etc. No flag is set. | */
    MANUAL_DISARMED=64, /* System is allowed to be active, under manual (RC) control, no stabilization (MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) | */
    TEST_DISARMED=66, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. (MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, MAV_MODE_FLAG_TEST_ENABLED). | */
    STABILIZE_DISARMED=80, /* System is allowed to be active, under assisted RC control (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_STABILIZE_ENABLED) | */
    GUIDED_DISARMED=88, /* System is allowed to be active, under autonomous control, manual setpoint (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_STABILIZE_ENABLED, MAV_MODE_FLAG_GUIDED_ENABLED) | */
    AUTO_DISARMED=92, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints). (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_STABILIZE_ENABLED, MAV_MODE_FLAG_GUIDED_ENABLED, MAV_MODE_FLAG_AUTO_ENABLED). | */
    MANUAL_ARMED=192, /* System is allowed to be active, under manual (RC) control, no stabilization (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) | */
    TEST_ARMED=194, /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, MAV_MODE_FLAG_TEST_ENABLED) | */
    STABILIZE_ARMED=208, /* System is allowed to be active, under assisted RC control (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, MAV_MODE_FLAG_STABILIZE_ENABLED) | */
    GUIDED_ARMED=216, /* System is allowed to be active, under autonomous control, manual setpoint (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, MAV_MODE_FLAG_STABILIZE_ENABLED, MAV_MODE_FLAG_GUIDED_ENABLED) | */
    AUTO_ARMED=220, /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints). (MAV_MODE_FLAG_SAFETY_ARMED, MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, MAV_MODE_FLAG_STABILIZE_ENABLED, MAV_MODE_FLAG_GUIDED_ENABLED,MAV_MODE_FLAG_AUTO_ENABLED). | */
};

//! MAV_MODE ENUM_END
constexpr auto MAV_MODE_ENUM_END = 221;

/** @brief These encode the sensors whose status is sent as part of the SYS_STATUS message. */
enum class MAV_SYS_STATUS_SENSOR : uint32_t
{
    SENSOR_3D_GYRO=1, /* 0x01 3D gyro | */
    SENSOR_3D_ACCEL=2, /* 0x02 3D accelerometer | */
    SENSOR_3D_MAG=4, /* 0x04 3D magnetometer | */
    ABSOLUTE_PRESSURE=8, /* 0x08 absolute pressure | */
    DIFFERENTIAL_PRESSURE=16, /* 0x10 differential pressure | */
    GPS=32, /* 0x20 GPS | */
    OPTICAL_FLOW=64, /* 0x40 optical flow | */
    VISION_POSITION=128, /* 0x80 computer vision position | */
    LASER_POSITION=256, /* 0x100 laser based position | */
    EXTERNAL_GROUND_TRUTH=512, /* 0x200 external ground truth (Vicon or Leica) | */
    ANGULAR_RATE_CONTROL=1024, /* 0x400 3D angular rate control | */
    ATTITUDE_STABILIZATION=2048, /* 0x800 attitude stabilization | */
    YAW_POSITION=4096, /* 0x1000 yaw position | */
    Z_ALTITUDE_CONTROL=8192, /* 0x2000 z/altitude control | */
    XY_POSITION_CONTROL=16384, /* 0x4000 x/y position control | */
    MOTOR_OUTPUTS=32768, /* 0x8000 motor outputs / control | */
    RC_RECEIVER=65536, /* 0x10000 RC receiver | */
    SENSOR_3D_GYRO2=131072, /* 0x20000 2nd 3D gyro | */
    SENSOR_3D_ACCEL2=262144, /* 0x40000 2nd 3D accelerometer | */
    SENSOR_3D_MAG2=524288, /* 0x80000 2nd 3D magnetometer | */
    GEOFENCE=1048576, /* 0x100000 geofence | */
    AHRS=2097152, /* 0x200000 AHRS subsystem health | */
    TERRAIN=4194304, /* 0x400000 Terrain subsystem health | */
    REVERSE_MOTOR=8388608, /* 0x800000 Motors are reversed | */
    LOGGING=16777216, /* 0x1000000 Logging | */
    BATTERY=33554432, /* 0x2000000 Battery | */
    PROXIMITY=67108864, /* 0x4000000 Proximity | */
    SATCOM=134217728, /* 0x8000000 Satellite Communication  | */
    PREARM_CHECK=268435456, /* 0x10000000 pre-arm check status. Always healthy when armed | */
    OBSTACLE_AVOIDANCE=536870912, /* 0x20000000 Avoidance/collision prevention | */
    PROPULSION=1073741824, /* 0x40000000 propulsion (actuator, esc, motor or propellor) | */
    EXTENSION_USED=2147483648, /* 0x80000000 Extended bit-field are used for further sensor status bits (needs to be set in onboard_control_sensors_present only) | */
};

//! MAV_SYS_STATUS_SENSOR ENUM_END
constexpr auto MAV_SYS_STATUS_SENSOR_ENUM_END = 2147483649;

/** @brief These encode the sensors whose status is sent as part of the SYS_STATUS message in the extended fields. */
enum class MAV_SYS_STATUS_SENSOR_EXTENDED : uint32_t
{
    RECOVERY_SYSTEM=1, /* 0x01 Recovery system (parachute, balloon, retracts etc) | */
    LEAK=2, /* 0x02 Leak detection | */
    EXTENDED_3D_GYRO3=4, /* 0x04 3rd 3D gyro | */
    EXTENDED_3D_ACCEL3=8, /* 0x08 3rd 3D accelerometer | */
    EXTENDED_3D_GYRO4=16, /* 0x10 4th 3D gyro | */
    EXTENDED_3D_ACCEL4=32, /* 0x20 4th 3D accelerometer | */
    EXTENDED_3D_MAG3=64, /* 0x40 3rd 3D magnetometer | */
    EXTENDED_3D_MAG4=128, /* 0x80 4th 3D magnetometer | */
};

//! MAV_SYS_STATUS_SENSOR_EXTENDED ENUM_END
constexpr auto MAV_SYS_STATUS_SENSOR_EXTENDED_ENUM_END = 129;

/** @brief Coordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or vehicles.

      Global frames use the following naming conventions:
      - "GLOBAL": Global coordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default.
        The following modifiers may be used with "GLOBAL":
        - "RELATIVE_ALT": Altitude is relative to the vehicle home position rather than MSL.
        - "TERRAIN_ALT": Altitude is relative to ground level rather than MSL.
        - "INT": Latitude/longitude (in degrees) are scaled by multiplying by 1E7.

      Local frames use the following naming conventions:
      - "LOCAL": Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ("EKF").
      - "BODY": Origin of local frame travels with the vehicle. NOTE, "BODY" does NOT indicate alignment of frame axis with vehicle attitude.
      - "OFFSET": Deprecated synonym for "BODY" (origin travels with the vehicle). Not to be used for new frames.

      Some deprecated frames do not follow these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).
  */
enum class MAV_FRAME : uint8_t
{
    GLOBAL=0, /* Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL). | */
    LOCAL_NED=1, /* NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth. | */
    MISSION=2, /* NOT a coordinate frame, indicates a mission command. | */
    GLOBAL_RELATIVE_ALT=3, /* 
          Global (WGS84) coordinate frame + altitude relative to the home position.
         | */
    LOCAL_ENU=4, /* ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth. | */
    GLOBAL_INT=5, /* Global (WGS84) coordinate frame (scaled) + altitude relative to mean sea level (MSL). | */
    GLOBAL_RELATIVE_ALT_INT=6, /* Global (WGS84) coordinate frame (scaled) + altitude relative to the home position.  | */
    LOCAL_OFFSET_NED=7, /* NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle. | */
    BODY_NED=8, /* Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/acceleration values. | */
    BODY_OFFSET_NED=9, /* This is the same as MAV_FRAME_BODY_FRD. | */
    GLOBAL_TERRAIN_ALT=10, /* Global (WGS84) coordinate frame with AGL altitude (altitude at ground level). | */
    GLOBAL_TERRAIN_ALT_INT=11, /* Global (WGS84) coordinate frame (scaled) with AGL altitude (altitude at ground level). | */
    BODY_FRD=12, /* FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle. | */
    RESERVED_13=13, /* MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up). | */
    RESERVED_14=14, /* MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down). | */
    RESERVED_15=15, /* MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up). | */
    RESERVED_16=16, /* MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down). | */
    RESERVED_17=17, /* MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up). | */
    RESERVED_18=18, /* MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down). | */
    RESERVED_19=19, /* MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up). | */
    LOCAL_FRD=20, /* FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane. | */
    LOCAL_FLU=21, /* FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane. | */
};

//! MAV_FRAME ENUM_END
constexpr auto MAV_FRAME_ENUM_END = 22;

/** @brief  */
enum class MAVLINK_DATA_STREAM_TYPE : uint8_t
{
    IMG_JPEG=0, /*  | */
    IMG_BMP=1, /*  | */
    IMG_RAW8U=2, /*  | */
    IMG_RAW32U=3, /*  | */
    IMG_PGM=4, /*  | */
    IMG_PNG=5, /*  | */
};

//! MAVLINK_DATA_STREAM_TYPE ENUM_END
constexpr auto MAVLINK_DATA_STREAM_TYPE_ENUM_END = 6;

/** @brief  */
enum class FENCE_BREACH : uint8_t
{
    NONE=0, /* No last fence breach | */
    MINALT=1, /* Breached minimum altitude | */
    MAXALT=2, /* Breached maximum altitude | */
    BOUNDARY=3, /* Breached fence boundary | */
};

//! FENCE_BREACH ENUM_END
constexpr auto FENCE_BREACH_ENUM_END = 4;

/** @brief Actions being taken to mitigate/prevent fence breach */
enum class FENCE_MITIGATE : uint8_t
{
    UNKNOWN=0, /* Unknown | */
    NONE=1, /* No actions being taken | */
    VEL_LIMIT=2, /* Velocity limiting active to prevent breach | */
};

//! FENCE_MITIGATE ENUM_END
constexpr auto FENCE_MITIGATE_ENUM_END = 3;

/** @brief Fence types to enable or disable when using MAV_CMD_DO_FENCE_ENABLE.
        Note that at least one of these flags must be set in MAV_CMD_DO_FENCE_ENABLE.param2.
        If none are set, the flight stack will ignore the field and enable/disable its default set of fences (usually all of them).
       */
enum class FENCE_TYPE
{
    ALT_MAX=1, /* Maximum altitude fence | */
    CIRCLE=2, /* Circle fence | */
    POLYGON=4, /* Polygon fence | */
    ALT_MIN=8, /* Minimum altitude fence | */
};

//! FENCE_TYPE ENUM_END
constexpr auto FENCE_TYPE_ENUM_END = 9;

/** @brief Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages. */
enum class MAV_MOUNT_MODE
{
    RETRACT=0, /* Load and keep safe position (Roll,Pitch,Yaw) from permanent memory and stop stabilization | */
    NEUTRAL=1, /* Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory. | */
    MAVLINK_TARGETING=2, /* Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization | */
    RC_TARGETING=3, /* Load neutral position and start RC Roll,Pitch,Yaw control with stabilization | */
    GPS_POINT=4, /* Load neutral position and start to point to Lat,Lon,Alt | */
    SYSID_TARGET=5, /* Gimbal tracks system with specified system ID | */
    HOME_LOCATION=6, /* Gimbal tracks home position | */
    WPNEXT_OFFSET=7, /* Gimbal tracks next waypoint location with offset | */
};

//! MAV_MOUNT_MODE ENUM_END
constexpr auto MAV_MOUNT_MODE_ENUM_END = 8;

/** @brief Gimbal device (low level) capability flags (bitmap). */
enum class GIMBAL_DEVICE_CAP_FLAGS : uint32_t
{
    HAS_RETRACT=1, /* Gimbal device supports a retracted position. | */
    HAS_NEUTRAL=2, /* Gimbal device supports a horizontal, forward looking position, stabilized. | */
    HAS_ROLL_AXIS=4, /* Gimbal device supports rotating around roll axis. | */
    HAS_ROLL_FOLLOW=8, /* Gimbal device supports to follow a roll angle relative to the vehicle. | */
    HAS_ROLL_LOCK=16, /* Gimbal device supports locking to a roll angle (generally that's the default with roll stabilized). | */
    HAS_PITCH_AXIS=32, /* Gimbal device supports rotating around pitch axis. | */
    HAS_PITCH_FOLLOW=64, /* Gimbal device supports to follow a pitch angle relative to the vehicle. | */
    HAS_PITCH_LOCK=128, /* Gimbal device supports locking to a pitch angle (generally that's the default with pitch stabilized). | */
    HAS_YAW_AXIS=256, /* Gimbal device supports rotating around yaw axis. | */
    HAS_YAW_FOLLOW=512, /* Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default). | */
    HAS_YAW_LOCK=1024, /* Gimbal device supports locking to an absolute heading, i.e., yaw angle relative to North (earth frame, often this is an option available). | */
    SUPPORTS_INFINITE_YAW=2048, /* Gimbal device supports yawing/panning infinitely (e.g. using slip disk). | */
    SUPPORTS_YAW_IN_EARTH_FRAME=4096, /* Gimbal device supports yaw angles and angular velocities relative to North (earth frame). This usually requires support by an autopilot via AUTOPILOT_STATE_FOR_GIMBAL_DEVICE. Support can go on and off during runtime, which is reported by the flag GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME. | */
    HAS_RC_INPUTS=8192, /* Gimbal device supports radio control inputs as an alternative input for controlling the gimbal orientation. | */
    CAN_POINT_LOCATION_LOCAL=65536, /* Gimbal device supports to point to a local position. | */
    CAN_POINT_LOCATION_GLOBAL=131072, /* Gimbal device supports to point to a global latitude, longitude, altitude position. | */
};

//! GIMBAL_DEVICE_CAP_FLAGS ENUM_END
constexpr auto GIMBAL_DEVICE_CAP_FLAGS_ENUM_END = 131073;

/** @brief Gimbal manager high level capability flags (bitmap). The flags are identical to the GIMBAL_DEVICE_CAP_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add flags. */
enum class GIMBAL_MANAGER_CAP_FLAGS : uint32_t
{
    HAS_RETRACT=1, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT. | */
    HAS_NEUTRAL=2, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL. | */
    HAS_ROLL_AXIS=4, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS. | */
    HAS_ROLL_FOLLOW=8, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW. | */
    HAS_ROLL_LOCK=16, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK. | */
    HAS_PITCH_AXIS=32, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS. | */
    HAS_PITCH_FOLLOW=64, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW. | */
    HAS_PITCH_LOCK=128, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK. | */
    HAS_YAW_AXIS=256, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS. | */
    HAS_YAW_FOLLOW=512, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW. | */
    HAS_YAW_LOCK=1024, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK. | */
    SUPPORTS_INFINITE_YAW=2048, /* Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW. | */
    SUPPORTS_YAW_IN_EARTH_FRAME=4096, /* Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME. | */
    HAS_RC_INPUTS=8192, /* Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS. | */
    CAN_POINT_LOCATION_LOCAL=65536, /* Based on GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL. | */
    CAN_POINT_LOCATION_GLOBAL=131072, /* Based on GIMBAL_DEVICE_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL. | */
};

//! GIMBAL_MANAGER_CAP_FLAGS ENUM_END
constexpr auto GIMBAL_MANAGER_CAP_FLAGS_ENUM_END = 131073;

/** @brief Flags for gimbal device (lower level) operation. */
enum class GIMBAL_DEVICE_FLAGS : uint16_t
{
    RETRACT=1, /* Set to retracted safe position (no stabilization), takes precedence over all other flags. | */
    NEUTRAL=2, /* Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (roll=pitch=yaw=0) but may be any orientation. | */
    ROLL_LOCK=4, /* Lock roll angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal. | */
    PITCH_LOCK=8, /* Lock pitch angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal. | */
    YAW_LOCK=16, /* Lock yaw angle to absolute angle relative to North (not relative to vehicle). If this flag is set, the yaw angle and z component of angular velocity are relative to North (earth frame, x-axis pointing North), else they are relative to the vehicle heading (vehicle frame, earth frame rotated so that the x-axis is pointing forward). | */
    YAW_IN_VEHICLE_FRAME=32, /* Yaw angle and z component of angular velocity are relative to the vehicle heading (vehicle frame, earth frame rotated such that the x-axis is pointing forward). | */
    YAW_IN_EARTH_FRAME=64, /* Yaw angle and z component of angular velocity are relative to North (earth frame, x-axis is pointing North). | */
    ACCEPTS_YAW_IN_EARTH_FRAME=128, /* Gimbal device can accept yaw angle inputs relative to North (earth frame). This flag is only for reporting (attempts to set this flag are ignored). | */
    RC_EXCLUSIVE=256, /* The gimbal orientation is set exclusively by the RC signals feed to the gimbal's radio control inputs. MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE) are ignored. | */
    RC_MIXED=512, /* The gimbal orientation is determined by combining/mixing the RC signals feed to the gimbal's radio control inputs and the MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE). How these two controls are combined or mixed is not defined by the protocol but is up to the implementation. | */
};

//! GIMBAL_DEVICE_FLAGS ENUM_END
constexpr auto GIMBAL_DEVICE_FLAGS_ENUM_END = 513;

/** @brief Flags for high level gimbal manager operation The first 16 bits are identical to the GIMBAL_DEVICE_FLAGS. */
enum class GIMBAL_MANAGER_FLAGS : uint32_t
{
    RETRACT=1, /* Based on GIMBAL_DEVICE_FLAGS_RETRACT. | */
    NEUTRAL=2, /* Based on GIMBAL_DEVICE_FLAGS_NEUTRAL. | */
    ROLL_LOCK=4, /* Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK. | */
    PITCH_LOCK=8, /* Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK. | */
    YAW_LOCK=16, /* Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK. | */
    YAW_IN_VEHICLE_FRAME=32, /* Based on GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME. | */
    YAW_IN_EARTH_FRAME=64, /* Based on GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME. | */
    ACCEPTS_YAW_IN_EARTH_FRAME=128, /* Based on GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME. | */
    RC_EXCLUSIVE=256, /* Based on GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE. | */
    RC_MIXED=512, /* Based on GIMBAL_DEVICE_FLAGS_RC_MIXED. | */
};

//! GIMBAL_MANAGER_FLAGS ENUM_END
constexpr auto GIMBAL_MANAGER_FLAGS_ENUM_END = 513;

/** @brief Gimbal device (low level) error flags (bitmap, 0 means no error) */
enum class GIMBAL_DEVICE_ERROR_FLAGS : uint32_t
{
    AT_ROLL_LIMIT=1, /* Gimbal device is limited by hardware roll limit. | */
    AT_PITCH_LIMIT=2, /* Gimbal device is limited by hardware pitch limit. | */
    AT_YAW_LIMIT=4, /* Gimbal device is limited by hardware yaw limit. | */
    ENCODER_ERROR=8, /* There is an error with the gimbal encoders. | */
    POWER_ERROR=16, /* There is an error with the gimbal power source. | */
    MOTOR_ERROR=32, /* There is an error with the gimbal motors. | */
    SOFTWARE_ERROR=64, /* There is an error with the gimbal's software. | */
    COMMS_ERROR=128, /* There is an error with the gimbal's communication. | */
    CALIBRATION_RUNNING=256, /* Gimbal device is currently calibrating. | */
    NO_MANAGER=512, /* Gimbal device is not assigned to a gimbal manager. | */
};

//! GIMBAL_DEVICE_ERROR_FLAGS ENUM_END
constexpr auto GIMBAL_DEVICE_ERROR_FLAGS_ENUM_END = 513;

/** @brief Gripper actions. */
enum class GRIPPER_ACTIONS
{
    ACTION_RELEASE=0, /* Gripper release cargo. | */
    ACTION_GRAB=1, /* Gripper grab onto cargo. | */
    ACTION_HOLD=2, /* Gripper hold current grip state/position. | */
};

//! GRIPPER_ACTIONS ENUM_END
constexpr auto GRIPPER_ACTIONS_ENUM_END = 3;

/** @brief Winch actions. */
enum class WINCH_ACTIONS
{
    RELAXED=0, /* Allow motor to freewheel. | */
    RELATIVE_LENGTH_CONTROL=1, /* Wind or unwind specified length of line, optionally using specified rate. | */
    RATE_CONTROL=2, /* Wind or unwind line at specified rate. | */
    LOCK=3, /* Perform the locking sequence to relieve motor while in the fully retracted position. Only action and instance command parameters are used, others are ignored. | */
    DELIVER=4, /* Sequence of drop, slow down, touch down, reel up, lock. Only action and instance command parameters are used, others are ignored. | */
    HOLD=5, /* Engage motor and hold current position. Only action and instance command parameters are used, others are ignored. | */
    RETRACT=6, /* Return the reel to the fully retracted position. Only action and instance command parameters are used, others are ignored. | */
    LOAD_LINE=7, /* Load the reel with line. The winch will calculate the total loaded length and stop when the tension exceeds a threshold. Only action and instance command parameters are used, others are ignored. | */
    ABANDON_LINE=8, /* Spool out the entire length of the line. Only action and instance command parameters are used, others are ignored. | */
    LOAD_PAYLOAD=9, /* Spools out just enough to present the hook to the user to load the payload. Only action and instance command parameters are used, others are ignored | */
};

//! WINCH_ACTIONS ENUM_END
constexpr auto WINCH_ACTIONS_ENUM_END = 10;

/** @brief Generalized UAVCAN node health */
enum class UAVCAN_NODE_HEALTH : uint8_t
{
    OK=0, /* The node is functioning properly. | */
    WARNING=1, /* A critical parameter went out of range or the node has encountered a minor failure. | */
    ERROR=2, /* The node has encountered a major failure. | */
    CRITICAL=3, /* The node has suffered a fatal malfunction. | */
};

//! UAVCAN_NODE_HEALTH ENUM_END
constexpr auto UAVCAN_NODE_HEALTH_ENUM_END = 4;

/** @brief Generalized UAVCAN node mode */
enum class UAVCAN_NODE_MODE : uint8_t
{
    OPERATIONAL=0, /* The node is performing its primary functions. | */
    INITIALIZATION=1, /* The node is initializing; this mode is entered immediately after startup. | */
    MAINTENANCE=2, /* The node is under maintenance. | */
    SOFTWARE_UPDATE=3, /* The node is in the process of updating its software. | */
    OFFLINE=7, /* The node is no longer available online. | */
};

//! UAVCAN_NODE_MODE ENUM_END
constexpr auto UAVCAN_NODE_MODE_ENUM_END = 8;

/** @brief Indicates the ESC connection type. */
enum class ESC_CONNECTION_TYPE : uint8_t
{
    PPM=0, /* Traditional PPM ESC. | */
    SERIAL=1, /* Serial Bus connected ESC. | */
    ONESHOT=2, /* One Shot PPM ESC. | */
    I2C=3, /* I2C ESC. | */
    CAN=4, /* CAN-Bus ESC. | */
    DSHOT=5, /* DShot ESC. | */
};

//! ESC_CONNECTION_TYPE ENUM_END
constexpr auto ESC_CONNECTION_TYPE_ENUM_END = 6;

/** @brief Flags to report ESC failures. */
enum class ESC_FAILURE_FLAGS : uint16_t
{
    OVER_CURRENT=1, /* Over current failure. | */
    OVER_VOLTAGE=2, /* Over voltage failure. | */
    OVER_TEMPERATURE=4, /* Over temperature failure. | */
    OVER_RPM=8, /* Over RPM failure. | */
    INCONSISTENT_CMD=16, /* Inconsistent command failure i.e. out of bounds. | */
    MOTOR_STUCK=32, /* Motor stuck failure. | */
    GENERIC=64, /* Generic ESC failure. | */
};

//! ESC_FAILURE_FLAGS ENUM_END
constexpr auto ESC_FAILURE_FLAGS_ENUM_END = 65;

/** @brief Flags to indicate the status of camera storage. */
enum class STORAGE_STATUS : uint8_t
{
    EMPTY=0, /* Storage is missing (no microSD card loaded for example.) | */
    UNFORMATTED=1, /* Storage present but unformatted. | */
    READY=2, /* Storage present and ready. | */
    NOT_SUPPORTED=3, /* Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored. | */
};

//! STORAGE_STATUS ENUM_END
constexpr auto STORAGE_STATUS_ENUM_END = 4;

/** @brief Flags to indicate the type of storage. */
enum class STORAGE_TYPE : uint8_t
{
    UNKNOWN=0, /* Storage type is not known. | */
    USB_STICK=1, /* Storage type is USB device. | */
    SD=2, /* Storage type is SD card. | */
    MICROSD=3, /* Storage type is microSD card. | */
    CF=4, /* Storage type is CFast. | */
    CFE=5, /* Storage type is CFexpress. | */
    XQD=6, /* Storage type is XQD. | */
    HD=7, /* Storage type is HD mass storage type. | */
    OTHER=254, /* Storage type is other, not listed type. | */
};

//! STORAGE_TYPE ENUM_END
constexpr auto STORAGE_TYPE_ENUM_END = 255;

/** @brief Flags to indicate usage for a particular storage (see STORAGE_INFORMATION.storage_usage and MAV_CMD_SET_STORAGE_USAGE). */
enum class STORAGE_USAGE_FLAG : uint8_t
{
    SET=1, /* Always set to 1 (indicates STORAGE_INFORMATION.storage_usage is supported). | */
    PHOTO=2, /* Storage for saving photos. | */
    VIDEO=4, /* Storage for saving videos. | */
    LOGS=8, /* Storage for saving logs. | */
};

//! STORAGE_USAGE_FLAG ENUM_END
constexpr auto STORAGE_USAGE_FLAG_ENUM_END = 9;

/** @brief Yaw behaviour during orbit flight. */
enum class ORBIT_YAW_BEHAVIOUR
{
    HOLD_FRONT_TO_CIRCLE_CENTER=0, /* Vehicle front points to the center (default). | */
    HOLD_INITIAL_HEADING=1, /* Vehicle front holds heading when message received. | */
    UNCONTROLLED=2, /* Yaw uncontrolled. | */
    HOLD_FRONT_TANGENT_TO_CIRCLE=3, /* Vehicle front follows flight path (tangential to circle). | */
    RC_CONTROLLED=4, /* Yaw controlled by RC input. | */
    UNCHANGED=5, /* Vehicle uses current yaw behaviour (unchanged). The vehicle-default yaw behaviour is used if this value is specified when orbit is first commanded. | */
};

//! ORBIT_YAW_BEHAVIOUR ENUM_END
constexpr auto ORBIT_YAW_BEHAVIOUR_ENUM_END = 6;

/** @brief Possible responses from a WIFI_CONFIG_AP message. */
enum class WIFI_CONFIG_AP_RESPONSE : int8_t
{
    UNDEFINED=0, /* Undefined response. Likely an indicative of a system that doesn't support this request. | */
    ACCEPTED=1, /* Changes accepted. | */
    REJECTED=2, /* Changes rejected. | */
    MODE_ERROR=3, /* Invalid Mode. | */
    SSID_ERROR=4, /* Invalid SSID. | */
    PASSWORD_ERROR=5, /* Invalid Password. | */
};

//! WIFI_CONFIG_AP_RESPONSE ENUM_END
constexpr auto WIFI_CONFIG_AP_RESPONSE_ENUM_END = 6;

/** @brief Possible responses from a CELLULAR_CONFIG message. */
enum class CELLULAR_CONFIG_RESPONSE : uint8_t
{
    ACCEPTED=0, /* Changes accepted. | */
    APN_ERROR=1, /* Invalid APN. | */
    PIN_ERROR=2, /* Invalid PIN. | */
    REJECTED=3, /* Changes rejected. | */
    BLOCKED_PUK_REQUIRED=4, /* PUK is required to unblock SIM card. | */
};

//! CELLULAR_CONFIG_RESPONSE ENUM_END
constexpr auto CELLULAR_CONFIG_RESPONSE_ENUM_END = 5;

/** @brief WiFi Mode. */
enum class WIFI_CONFIG_AP_MODE : int8_t
{
    UNDEFINED=0, /* WiFi mode is undefined. | */
    AP=1, /* WiFi configured as an access point. | */
    STATION=2, /* WiFi configured as a station connected to an existing local WiFi network. | */
    DISABLED=3, /* WiFi disabled. | */
};

//! WIFI_CONFIG_AP_MODE ENUM_END
constexpr auto WIFI_CONFIG_AP_MODE_ENUM_END = 4;

/** @brief Supported component metadata types. These are used in the "general" metadata file returned by COMPONENT_METADATA to provide information about supported metadata types. The types are not used directly in MAVLink messages. */
enum class COMP_METADATA_TYPE
{
    GENERAL=0, /* General information about the component. General metadata includes information about other metadata types supported by the component. Files of this type must be supported, and must be downloadable from vehicle using a MAVLink FTP URI. | */
    PARAMETER=1, /* Parameter meta data. | */
    COMMANDS=2, /* Meta data that specifies which commands and command parameters the vehicle supports. (WIP) | */
    PERIPHERALS=3, /* Meta data that specifies external non-MAVLink peripherals. | */
    EVENTS=4, /* Meta data for the events interface. | */
    ACTUATORS=5, /* Meta data for actuator configuration (motors, servos and vehicle geometry) and testing. | */
};

//! COMP_METADATA_TYPE ENUM_END
constexpr auto COMP_METADATA_TYPE_ENUM_END = 6;

/** @brief Actuator configuration, used to change a setting on an actuator. Component information metadata can be used to know which outputs support which commands. */
enum class ACTUATOR_CONFIGURATION
{
    NONE=0, /* Do nothing. | */
    BEEP=1, /* Command the actuator to beep now. | */
    CONFIGURATION_3D_MODE_ON=2, /* Permanently set the actuator (ESC) to 3D mode (reversible thrust). | */
    CONFIGURATION_3D_MODE_OFF=3, /* Permanently set the actuator (ESC) to non 3D mode (non-reversible thrust). | */
    SPIN_DIRECTION1=4, /* Permanently set the actuator (ESC) to spin direction 1 (which can be clockwise or counter-clockwise). | */
    SPIN_DIRECTION2=5, /* Permanently set the actuator (ESC) to spin direction 2 (opposite of direction 1). | */
};

//! ACTUATOR_CONFIGURATION ENUM_END
constexpr auto ACTUATOR_CONFIGURATION_ENUM_END = 6;

/** @brief Actuator output function. Values greater or equal to 1000 are autopilot-specific. */
enum class ACTUATOR_OUTPUT_FUNCTION
{
    NONE=0, /* No function (disabled). | */
    MOTOR1=1, /* Motor 1 | */
    MOTOR2=2, /* Motor 2 | */
    MOTOR3=3, /* Motor 3 | */
    MOTOR4=4, /* Motor 4 | */
    MOTOR5=5, /* Motor 5 | */
    MOTOR6=6, /* Motor 6 | */
    MOTOR7=7, /* Motor 7 | */
    MOTOR8=8, /* Motor 8 | */
    MOTOR9=9, /* Motor 9 | */
    MOTOR10=10, /* Motor 10 | */
    MOTOR11=11, /* Motor 11 | */
    MOTOR12=12, /* Motor 12 | */
    MOTOR13=13, /* Motor 13 | */
    MOTOR14=14, /* Motor 14 | */
    MOTOR15=15, /* Motor 15 | */
    MOTOR16=16, /* Motor 16 | */
    SERVO1=33, /* Servo 1 | */
    SERVO2=34, /* Servo 2 | */
    SERVO3=35, /* Servo 3 | */
    SERVO4=36, /* Servo 4 | */
    SERVO5=37, /* Servo 5 | */
    SERVO6=38, /* Servo 6 | */
    SERVO7=39, /* Servo 7 | */
    SERVO8=40, /* Servo 8 | */
    SERVO9=41, /* Servo 9 | */
    SERVO10=42, /* Servo 10 | */
    SERVO11=43, /* Servo 11 | */
    SERVO12=44, /* Servo 12 | */
    SERVO13=45, /* Servo 13 | */
    SERVO14=46, /* Servo 14 | */
    SERVO15=47, /* Servo 15 | */
    SERVO16=48, /* Servo 16 | */
};

//! ACTUATOR_OUTPUT_FUNCTION ENUM_END
constexpr auto ACTUATOR_OUTPUT_FUNCTION_ENUM_END = 49;

/** @brief Axes that will be autotuned by MAV_CMD_DO_AUTOTUNE_ENABLE.
        Note that at least one flag must be set in MAV_CMD_DO_AUTOTUNE_ENABLE.param2: if none are set, the flight stack will tune its default set of axes. */
enum class AUTOTUNE_AXIS
{
    ROLL=1, /* Autotune roll axis. | */
    PITCH=2, /* Autotune pitch axis. | */
    YAW=4, /* Autotune yaw axis. | */
};

//! AUTOTUNE_AXIS ENUM_END
constexpr auto AUTOTUNE_AXIS_ENUM_END = 5;

/** @brief 
        Actions for reading/writing parameters between persistent and volatile storage when using MAV_CMD_PREFLIGHT_STORAGE.
        (Commonly parameters are loaded from persistent storage (flash/EEPROM) into volatile storage (RAM) on startup and written back when they are changed.)
       */
enum class PREFLIGHT_STORAGE_PARAMETER_ACTION
{
    PARAM_READ_PERSISTENT=0, /* Read all parameters from persistent storage. Replaces values in volatile storage. | */
    PARAM_WRITE_PERSISTENT=1, /* Write all parameter values to persistent storage (flash/EEPROM) | */
    PARAM_RESET_FACTORY_DEFAULT=2, /* Reset parameters to default values (such as sensor calibration, safety settings, and so on). Note that a flight stack may choose not to reset some parameters at their own discretion (such as those that are locked or expected to persist for the vehicle lifetime). | */
    PARAM_RESET_SENSOR_DEFAULT=3, /* Reset only sensor calibration parameters to factory defaults (or firmware default if not available) | */
    PARAM_RESET_ALL_DEFAULT=4, /* Reset all parameters to default values. | */
};

//! PREFLIGHT_STORAGE_PARAMETER_ACTION ENUM_END
constexpr auto PREFLIGHT_STORAGE_PARAMETER_ACTION_ENUM_END = 5;

/** @brief Reboot/shutdown action for selected component in MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN. */
enum class REBOOT_SHUTDOWN_ACTION
{
    NONE=0, /* Do nothing. | */
    REBOOT=1, /* Reboot component. | */
    SHUTDOWN=2, /* Shutdown component. | */
    REBOOT_TO_BOOTLOADER=3, /* Reboot component and keep it in the bootloader until upgraded. | */
    POWER_ON=4, /* Power on component. Do nothing if component is already powered (ACK command with MAV_RESULT_ACCEPTED). | */
    REBOOT_TO_MASS_STORAGE=5, /* Reboot component into a mass storage mode. | */
};

//! REBOOT_SHUTDOWN_ACTION ENUM_END
constexpr auto REBOOT_SHUTDOWN_ACTION_ENUM_END = 6;

/** @brief Specifies the conditions under which the MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN command should be accepted. */
enum class REBOOT_SHUTDOWN_CONDITIONS
{
    SAFETY_INTERLOCKED=0, /* Reboot/Shutdown only if allowed by safety checks, such as being landed. | */
    FORCE=20190226, /* Force reboot/shutdown of the autopilot/component regardless of system state. | */
};

//! REBOOT_SHUTDOWN_CONDITIONS ENUM_END
constexpr auto REBOOT_SHUTDOWN_CONDITIONS_ENUM_END = 20190227;

/** @brief Action for the magnetometer (param2) of MAV_CMD_PREFLIGHT_CALIBRATION. */
enum class PREFLIGHT_CALIBRATION_MAGNETOMETER
{
    NONE=0, /* No action. | */
    START=1, /* Start magnetometer calibration. | */
    FORCE_SAVE=76, /* Force-accept the existing compass calibration as valid without re-running it. Useful after a parameter reload that cleared calibration validity flags. | */
};

//! PREFLIGHT_CALIBRATION_MAGNETOMETER ENUM_END
constexpr auto PREFLIGHT_CALIBRATION_MAGNETOMETER_ENUM_END = 77;

/** @brief Action for the accelerometer (param5) of MAV_CMD_PREFLIGHT_CALIBRATION. */
enum class PREFLIGHT_CALIBRATION_ACCELEROMETER
{
    NONE=0, /* No action. | */
    FULL=1, /* Full 6-position accelerometer calibration. | */
    TRIM=2, /* Board level (trim) calibration. | */
    TEMPERATURE=3, /* Accelerometer temperature calibration. | */
    SIMPLE=4, /* Simple accelerometer calibration. | */
    FORCE_SAVE=76, /* Force-accept the existing accelerometer calibration as valid without re-running it. Useful after a parameter reload that cleared calibration validity flags. | */
};

//! PREFLIGHT_CALIBRATION_ACCELEROMETER ENUM_END
constexpr auto PREFLIGHT_CALIBRATION_ACCELEROMETER_ENUM_END = 77;

/** @brief  */
enum class NAV_TAKEOFF_FLAGS
{
    HORIZONTAL_POSITION_NOT_REQUIRED=1, /* Accept the command even if the autopilot does not have control over its horizontal position (note that it might not have altitude control either). | */
};

//! NAV_TAKEOFF_FLAGS ENUM_END
constexpr auto NAV_TAKEOFF_FLAGS_ENUM_END = 2;

/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current yaw or latitude rather than a specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries */
enum class MAV_CMD : uint16_t
{
    NAV_WAYPOINT=16, /* Navigate to waypoint. This is intended for use in missions (for guided commands outside of missions use MAV_CMD_DO_REPOSITION). |Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)| 0 to pass through the WP, if > 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    NAV_LOITER_UNLIM=17, /* Loiter around this waypoint an unlimited amount of time |Empty| Empty| Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise| Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TURNS=18, /* Loiter around this waypoint for X turns |Number of turns.| Leave loiter circle only when track heads towards the next waypoint (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE causes leaving when turns complete. Values not equal to 0 or 1 are invalid.| Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise| Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. NaN to use the current system default xtrack behaviour.| Latitude| Longitude| Altitude|  */
    NAV_LOITER_TIME=19, /* Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint. |Loiter time (only starts once Lat, Lon and Alt is reached).| Leave loiter circle only when track heading towards the next waypoint (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE causes leaving on time expiry. Values not equal to 0 or 1 are invalid.| Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.| Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. NaN to use the current system default xtrack behaviour.| Latitude| Longitude| Altitude|  */
    NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_LAND=21, /* Land at location. |Minimum target altitude if landing is aborted (0 = undefined/use system default).| Precision land mode.| Empty.| Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude.| Longitude.| Landing altitude (ground level in current frame).|  */
    NAV_TAKEOFF=22, /* Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode. |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Bitmask of options flags.| Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |Landing target number (if available)| Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| Landing descend rate| Desired yaw angle| Y-axis position| X-axis position| Z-axis / ground level position|  */
    NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Takeoff ascend rate| Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these| Y-axis position| X-axis position| Z-axis position|  */
    NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.| Empty| Empty| Empty| Empty| Empty| Desired altitude|  */
    NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint. |Leave loiter circle only when track heading towards the next waypoint (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE causes leaving when altitude is reached. Values not equal to 0 or 1 are invalid.| Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| Empty| Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. NaN to use the current system default xtrack behaviour.| Latitude| Longitude| Altitude|  */
    DO_FOLLOW=32, /* Begin following a target |System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.| Reserved| Reserved| Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.| Altitude above home. (used if mode=2)| Reserved| Time to land in which the MAV should go to the default position hold mode after a message RX timeout.|  */
    DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |Camera q1 (where 0 is on the ray from the camera to the tracking device)| Camera q2| Camera q3| Camera q4| altitude offset from target| X offset from target| Y offset from target|  */
    DO_ORBIT=34, /* Start orbiting on the circumference of a circle defined by the parameters. Setting values to NaN/INT32_MAX (as appropriate) results in using defaults. |Radius of the circle. Positive: orbit clockwise. Negative: orbit counter-clockwise. NaN: Use vehicle default radius, or current radius if already orbiting.| Tangential Velocity. NaN: Use vehicle default velocity, or current velocity if already orbiting.| Yaw behavior of the vehicle.| Orbit around the centre point for this many radians (i.e. for a three-quarter orbit set 270*Pi/180). 0: Orbit forever. NaN: Use vehicle default, or current value if already orbiting.| Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. INT32_MAX (COMMAND_INT) or NaN (COMMAND_LONG): Use current vehicle position, or current center if already orbiting.| Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. INT32_MAX (COMMAND_INT) or NaN (COMMAND_LONG): Use current vehicle position, or current center if already orbiting.| Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle altitude.|  */
    DO_FIGURE_EIGHT=35, /* Fly a figure eight path as defined by the parameters.
          Set parameters to NaN/INT32_MAX (as appropriate) to use system-default values.
          The command is intended for fixed wing vehicles (and VTOL hybrids flying in fixed-wing mode), allowing POI tracking for gimbals that don't support infinite rotation.
          This command only defines the flight path. Speed should be set independently (use e.g. MAV_CMD_DO_CHANGE_SPEED).
          Yaw and other degrees of freedom are not specified, and will be flight-stack specific (on vehicles where they can be controlled independent of the heading).
         |Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise.
        NaN: The radius will be set to 2.5 times the minor radius and direction is clockwise.
        Must be greater or equal to two times the minor radius for feasible values.| Minor axis radius of the figure eight. Defines the radius of the two circles that make up the figure. Negative value has no effect.
        NaN: The radius will be set to the default loiter radius.| Reserved (default:NaN)| Orientation of the figure eight major axis with respect to true north (range: [-pi,pi]). NaN: use default orientation aligned to true north.| Center point latitude/X coordinate according to MAV_FRAME. If no MAV_FRAME specified, MAV_FRAME_GLOBAL is assumed.
        INT32_MAX (COMMAND_INT) or NaN (COMMAND_LONG): Use current vehicle position, or current center if already loitering.| Center point longitude/Y coordinate according to MAV_FRAME. If no MAV_FRAME specified, MAV_FRAME_GLOBAL is assumed.
        INT32_MAX (COMMAND_INT) or NaN (COMMAND_LONG): Use current vehicle position, or current center if already loitering.| Center point altitude MSL/Z coordinate according to MAV_FRAME. If no MAV_FRAME specified, MAV_FRAME_GLOBAL is assumed.
        INT32_MAX or NaN: Use current vehicle altitude.|  */
    NAV_ARC_WAYPOINT=36, /* Circular arc path waypoint.
          This defines the end/exit point and angle (param1) of an arc path from the previous waypoint. A position is required before this command to define the start of the arc (e.g. current position, a MAV_CMD_NAV_WAYPOINT, or a MAV_CMD_NAV_ARC_WAYPOINT).
          The resulting path is a circular arc in the NE frame, with the difference in height being defined by the difference in waypoint altitudes.
         |The angle in degrees from the starting position to the exit position of the arc in the NE frame. Positive values are CW arcs and negative values are CCW arcs.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Latitude| Longitude| Altitude|  */
    NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode.| Waypoint index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
    NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_SPLINE_WAYPOINT=82, /* Navigate to waypoint using a spline path. |Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| Empty| Empty| Empty| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
    NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.). |Empty| Front transition heading.| Empty| Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude|  */
    NAV_VTOL_LAND=85, /* Land using VTOL mode |Landing behaviour.| Empty| Approach altitude (with the same reference as the Altitude field). NaN if unspecified.| Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| Latitude| Longitude| Altitude (ground level) relative to the current coordinate frame. NaN to use system default landing altitude (ignore value).|  */
    NAV_GUIDED_ENABLE=92, /* Hand control over to an external controller |Guided mode on (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE turns guided mode off. Values not equal to 0 or 1 are invalid.| Empty| Empty| Empty| Empty| Empty| Empty|  */
    NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |Delay (-1 to enable time-of-day fields)| hour (24h format, UTC, -1 to ignore)| minute (24h format, UTC, -1 to ignore)| second (24h format, UTC, -1 to ignore)| Empty| Empty| Empty|  */
    NAV_PAYLOAD_PLACE=94, /* Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. |Maximum distance to descend.| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
    NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_DELAY=112, /* Delay mission state machine. |Delay| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_CHANGE_ALT=113, /* Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached. |Descent / Ascend rate.| Empty| Empty| Empty| Empty| Empty| Target Altitude|  */
    CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance.| Empty| Empty| Empty| Empty| Empty| Empty|  */
    CONDITION_YAW=115, /* Reach a certain target angle. |target angle [0-360]. Absolute angles: 0 is north. Relative angle: 0 is initial yaw. Direction set by param3.| angular speed| direction: -1: counter clockwise, 0: shortest direction, 1: clockwise| Relative offset (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE specifies an absolute angle. Values not equal to 0 or 1 are invalid.| Empty| Empty| Empty|  */
    CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_MODE=176, /* Set system mode. |Mode flags. MAV_MODE values can be used to set some mode flag combinations.| Custom system-specific mode (see target autopilot specifications for mode information). If MAV_MODE_FLAG_CUSTOM_MODE_ENABLED is set in param1 (mode) this mode is used: otherwise the field is ignored.| Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| Empty| Empty| Empty| Empty|  */
    DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. The value persists until it is overridden or there is a mode change |Speed type of value set in param2 (such as airspeed, ground speed, and so on)| Speed (-1 indicates no change, -2 indicates return to default vehicle speed)| Throttle (-1 indicates no change, -2 indicates return to default vehicle throttle value)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_SET_HOME=179, /* 
          Sets the home position to either to the current position or a specified position.
          The home position is the default position that the system will return to and land on.
          The position is set automatically by the system during the takeoff (and may also be set using this command).
          Note: the current home position may be emitted in a HOME_POSITION message on request (using MAV_CMD_REQUEST_MESSAGE with param1=242).
         |Use current location (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE uses the specified location. Values not equal to 0 or 1 are invalid.| Roll angle (of surface). Range: -180..180 degrees. NAN or 0 means value not set. 0.01 indicates zero roll.| Pitch angle (of surface). Range: -90..90 degrees. NAN or 0 means value not set. 0.01 means zero pitch.| Yaw angle. NaN to use default heading. Range: -180..180 degrees.| Latitude| Longitude| Altitude|  */
    DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_RELAY=181, /* Set a relay to a condition. The current value may optionally be reported using RELAY_STATUS. |Relay instance number.| Setting. (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cycles with a desired period. |Relay instance number.| Cycle count.| Cycle time.| Empty| Empty| Empty| Empty|  */
    DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |Servo instance number.| Pulse Width Modulation.| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo instance number.| Pulse Width Modulation.| Cycle count.| Cycle time.| Empty| Empty| Empty|  */
    DO_FLIGHTTERMINATION=185, /* Terminate flight immediately.
          Flight termination immediately and irreversibly terminates the current flight, returning the vehicle to ground.
          The vehicle will ignore RC or other input until it has been power-cycled.
          Termination may trigger safety measures, including: disabling motors and deployment of parachute on multicopters, and setting flight surfaces to initiate a landing pattern on fixed-wing).
          On multicopters without a parachute it may trigger a crash landing.
          Support for this command can be tested using the protocol bit: MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION.
          Support for this command can also be tested by sending the command with param1=0 (< 0.5); the ACK should be either MAV_RESULT_FAILED or MAV_RESULT_UNSUPPORTED.
         |Flight termination activated if > 0.5. Otherwise not activated and ACK with MAV_RESULT_FAILED.| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |Altitude.| Frame of new altitude.| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_ACTUATOR=187, /* Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter). |Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.| Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.| Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.| Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.| Actuator 5 value.
          If sent in COMMAND_LONG: value is scaled from [-1 to 1]. NaN to ignore.
          If sent in COMMAND_INT or MISSION_ITEM_INT: value is scaled by 1e7. INT32_MAX to ignore.| Actuator 6 value.
          If sent in COMMAND_LONG: value is scaled from [-1 to 1]. NaN to ignore.
          If sent in COMMAND_INT or MISSION_ITEM_INT: value is scaled by 1e7. INT32_MAX to ignore.| Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)|  */
    DO_RETURN_PATH_START=188, /* Mission item to specify the start of a failsafe/landing return-path segment (the end of the segment is the next MAV_CMD_DO_LAND_START item).
          A vehicle that is using missions for landing (e.g. in a return mode) will join the mission on the closest path of the return-path segment (instead of MAV_CMD_DO_LAND_START or the nearest waypoint).
          The main use case is to minimize the failsafe flight path in corridor missions, where the inbound/outbound paths are constrained (by geofences) to the same particular path.
          The MAV_CMD_DO_RETURN_PATH_START would be placed at the start of the return path.
          If a failsafe occurs on the outbound path the vehicle will move to the nearest point on the return path (which is parallel for this kind of mission), effectively turning round and following the shortest path to landing.
          If a failsafe occurs on the inbound path the vehicle is already on the return segment and will continue to landing.
          The Latitude/Longitude/Altitude are optional, and may be set to 0 if not needed.
          If specified, the item defines the waypoint at which the return segment starts.
          If sent using as a command, the vehicle will perform a mission landing (using the land segment if defined) or reject the command if mission landings are not supported, or no mission landing is defined. When used as a command any position information in the command is ignored.
         |Empty| Empty| Empty| Empty| Latitudee. 0: not used.| Longitudee. 0: not used.| Altitudee. 0: not used.|  */
    DO_LAND_START=189, /* Mission item to mark the start of a mission landing pattern, or a command to land with a mission landing pattern.

        When used in a mission, this is a marker for the start of a sequence of mission items that represent a landing pattern.
        It should be followed by a navigation item that defines the first waypoint of the landing sequence.
        The start marker positional params are used only for selecting what landing pattern to use if several are defined in the mission (the selected pattern will be the one with the marker position that is closest to the vehicle when a landing is commanded).
        If the marker item position has zero-values for latitude, longitude, and altitude, then landing pattern selection is instead based on the position of the first waypoint in the landing sequence.

	    When sent as a command it triggers a landing using a mission landing pattern.
	    The location parameters are not used in this case, and should be set to 0.
	 |Empty| Empty| Empty| Empty| Latitude for landing sequence selection, or 0 (see description). Ignored in commands (set 0).| Longitude for landing sequence selection, or 0 (see description). Ignored in commands (set 0).| Altitude for landing sequence selection, or 0 (see description). Ignored in commands (set 0).|  */
    DO_RALLY_LAND=190, /* Command to perform a landing from a rally point. |Break altitude| Landing speed| Empty| Empty| Empty| Empty| Empty|  */
    DO_GO_AROUND=191, /* Command to safely abort an autonomous landing. |Altitude| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. This command is intended for guided commands (for missions use MAV_CMD_NAV_WAYPOINT instead). |Ground speed, less than 0 (-1) for default| Bitmask of option flags.| Loiter radius for planes. Positive values only, direction is controlled by Yaw value. A value of zero or NaN is ignored. | Yaw heading (heading reference defined in Bitmask field). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| Latitude| Longitude| Altitude|  */
    DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |Continue mission (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE pauses the current mission or reposition command and holds current position (a VTOL capable vehicle should enter hover mode (multicopter and VTOL planes); a plane should loiter with the default loiter radius). Values not equal to 0 or 1 are invalid.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |Reverse direction (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE specifies forward direction. Values not equal to 0 or 1 are invalid.| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_ROI_LOCATION=195, /* Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| Empty| Empty| Empty| Latitude of ROI location| Longitude of ROI location| Altitude of ROI location|  */
    DO_SET_ROI_WPNEXT_OFFSET=196, /* Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. |Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| Empty| Empty| Empty| Pitch offset from next waypoint, positive pitching up| Roll offset from next waypoint, positive rolling to the right| Yaw offset from next waypoint, positive yawing to the right|  */
    DO_SET_ROI_NONE=197, /* Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position. |Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_SET_ROI_SYSID=198, /* Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. |System ID| Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_CONTROL_VIDEO=200, /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
    DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of interest mode.| Waypoint index/ target ID (depends on param 1).| Region of interest index. (allows a vehicle to manage multiple ROI's)| Empty| MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude| MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude| MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude|  */
    DO_DIGICAM_CONFIGURE=202, /* Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Modes: P, TV, AV, M, Etc.| Shutter speed: Divisor number for one second.| Aperture: F stop number.| ISO number e.g. 80, 100, 200, Etc.| Exposure type enumerator.| Command Identity.| Main engine cut-off time before camera trigger. (0 means no cut-off)|  */
    DO_DIGICAM_CONTROL=203, /* Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |Session control e.g. show/hide lens| Zoom's absolute position| Zooming step value to offset zoom from the current position| Focus Locking, Unlocking or Re-locking| Shooting Command| Command Identity| Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  */
    DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |Mount operation mode| Stabilize roll (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Stabilize pitch (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Stabilize yaw (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| Pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| Yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)|  */
    DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |pitch depending on mount mode (degrees or degrees/second depending on pitch input).| roll depending on mount mode (degrees or degrees/second depending on roll input).| yaw depending on mount mode (degrees or degrees/second depending on yaw input).| altitude depending on mount mode.| latitude, set if appropriate mount mode.| longitude, set if appropriate mount mode.| Mount mode.|  */
    DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. |Camera trigger distance. 0 to stop triggering.| Camera shutter integration time. -1 or 0 to ignore| Trigger camera once, immediately (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Empty| Empty| Empty|  */
    DO_FENCE_ENABLE=207, /* 
          Enable the geofence.
          This can be used in a mission or via the command protocol.
          The persistence/lifetime of the setting is undefined.
          Depending on flight stack implementation it may persist until superseded, or it may revert to a system default at the end of a mission.
          Flight stacks typically reset the setting to system defaults on reboot.
	 |enable? (0=disable, 1=enable, 2=disable_floor_only)| Fence types to enable or disable as a bitmask. 0: field is unused/all fences should be enabled or disabled (for compatibility reasons). Parameter is ignored if param1=2.| Empty| Empty| Empty| Empty| Empty|  */
    DO_PARACHUTE=208, /* Mission item/command to release a parachute or enable/disable auto release. |Action| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_MOTOR_TEST=209, /* Command to perform motor test. |Motor instance number (from 1 to max number of motors on the vehicle).| Throttle type (whether the Throttle Value in param3 is a percentage, PWM value, etc.)| Throttle value.| Timeout between tests that are run in sequence.| Motor count. Number of motors to test in sequence: 0/1=one motor, 2= two motors, etc. The Timeout (param4) is used between tests.| Motor test order.| Empty|  */
    DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight. |Inverted flight (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE specifies normal flight. Values not equal to 0 or 1 are invalid.| Empty| Empty| Empty| Empty| Empty| Empty|  */
    DO_GRIPPER=211, /* Mission command to operate a gripper. |Gripper ID. 1-6 for an autopilot connected gripper. In missions this may be set to 1-6 for an autopilot gripper, or the gripper component id for a MAVLink gripper. 0 targets all grippers.| Gripper action to perform.| Empty| Empty| Empty| Empty| Empty|  */
    DO_AUTOTUNE_ENABLE=212, /* Enable/disable autotune. |Enable autotune (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE disables autotune. Values not equal to 0 or 1 are invalid.| Specify axes for which autotuning is enabled/disabled. 0 indicates the field is unused (for compatibility reasons). If 0 the autopilot will follow its default behaviour, which is usually to tune all axes.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    NAV_SET_YAW_SPEED=213, /* Sets a desired vehicle turn angle and speed change. |Yaw angle to adjust steering by.| Speed.| Relative final angle (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE specifies an absolute angle. Values not equal to 0 or 1 are invalid.| Empty| Empty| Empty| Empty|  */
    DO_SET_CAM_TRIGG_INTERVAL=214, /* Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. |Camera trigger cycle time. -1 or 0 to ignore.| Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Empty| Empty| Empty| Empty|  */
    DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |quaternion param q1, w (1 in null-rotation)| quaternion param q2, x (0 in null-rotation)| quaternion param q3, y (0 in null-rotation)| quaternion param q4, z (0 in null-rotation)| Empty| Empty| Empty|  */
    DO_GUIDED_MASTER=221, /* set id of master controller |System ID| Component ID| Empty| Empty| Empty| Empty| Empty|  */
    DO_GUIDED_LIMITS=222, /* Set limits for external control |Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.| Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.| Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.| Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.| Empty| Empty| Empty|  */
    DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |Start engine (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE stops the engine. Values not equal to 0 or 1 are invalid.| Cold start engine (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE performs a warm start. Controls use of choke where applicable. Values not equal to 0 or 1 are invalid.| Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| A bitmask of options for engine control| Empty| Empty| Empty|  */
    DO_SET_MISSION_CURRENT=224, /* 
          Set the mission item with sequence number seq as the current item and emit MISSION_CURRENT (whether or not the mission number changed).
          If a mission is currently being executed, the system will continue to this new mission item on the shortest path, skipping any intermediate mission items.
	  Note that mission jump repeat counters are not reset unless param2 is set (see MAV_CMD_DO_JUMP param2).

          This command may trigger a mission state-machine change on some systems: for example from MISSION_STATE_NOT_STARTED or MISSION_STATE_PAUSED to MISSION_STATE_ACTIVE.
          If the system is in mission mode, on those systems this command might therefore start, restart or resume the mission.
          If the system is not in mission mode this command must not trigger a switch to mission mode.

          The mission may be "reset" using param2.
          Resetting sets jump counters to initial values (to reset counters without changing the current mission item set the param1 to `-1`).
          Resetting also explicitly changes a mission state of MISSION_STATE_COMPLETE to MISSION_STATE_PAUSED or MISSION_STATE_ACTIVE, potentially allowing it to resume when it is (next) in a mission mode.

	  The command will ACK with MAV_RESULT_FAILED if the sequence number is out of range (including if there is no mission item).
         |Mission sequence value to set. -1 for the current mission item (use to reset mission without changing current mission item).| Reset mission. A value of MAV_BOOL_TRUE resets jump counters to initial values and changes mission state "completed" to be "active" or "paused". Values not equal to 0 or 1 are invalid.| Empty| Empty| Empty| Empty| Empty|  */
    DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
    PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: gyro calibration, 3: gyro temperature calibration| Magnetometer calibration action.| Ground pressure calibration (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| 1: radio RC calibration, 2: RC trim calibration| Accelerometer calibration action.| 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 1: ESC calibration, 3: barometer temperature calibration|  */
    PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
    PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named). |1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Action to perform on the persistent parameter storage| Reserved| Onboard logging: 0: Ignore, 1: Start default rate logging, -1: Stop logging, > 1: logging rate (e.g. set to 1000 for 1000 Hz logging)| Reserved| Empty| Empty| Empty|  */
    PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |Action to take for autopilot.| Action to take for onboard computer.| Action to take for component specified in param4.| MAVLink Component ID targeted in param3 (0 for all components).| Reserved (set to 0)| Conditions under which reboot/shutdown is allowed.| WIP: ID (e.g. camera ID -1 for all IDs)|  */
    OVERRIDE_GOTO=252, /* Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. |MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.| MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.| Coordinate frame of hold point.| Desired yaw angle.| Latitude/X position.| Longitude/Y position.| Altitude/Z position.|  */
    OBLIQUE_SURVEY=260, /* Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera. |Camera trigger distance. 0 to stop triggering.| Camera shutter integration time. 0 to ignore| The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.| Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).| Angle limits that the camera can be rolled to left and right of center.| Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.| Empty|  */
    DO_SET_STANDARD_MODE=262, /* Enable the specified standard MAVLink mode.
          If the specified mode is not supported, the vehicle should ACK with MAV_RESULT_FAILED.
          See https://mavlink.io/en/services/standard_modes.html
         |The mode to set.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    MISSION_START=300, /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    ACTUATOR_TEST=310, /* Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of output functions, i.e. it is possible to test Motor1 independent from which output it is configured on. Autopilots must NACK this command with MAV_RESULT_TEMPORARILY_REJECTED while armed. |Output value: 1 means maximum positive output, 0 to center servos or minimum motor thrust (expected to spin), -1 for maximum negative (if not supported by the motors, i.e. motor is not reversible, smaller than 0 maps to NaN). And NaN maps to disarmed (stop the motors).| Timeout after which the test command expires and the output is restored to the previous value. A timeout has to be set for safety reasons. A timeout of 0 means to restore the previous value immediately.| Reserved (default:0)| Reserved (default:0)| Actuator Output function| Reserved (default:0)| Reserved (default:0)|  */
    CONFIGURE_ACTUATOR=311, /* Actuator configuration command. |Actuator configuration action| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Actuator Output function| Reserved (default:0)| Reserved (default:0)|  */
    COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |Arm (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE disarms. Values not equal to 0 or 1 are invalid.| 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    RUN_PREARM_CHECKS=401, /* Instructs a target system to run pre-arm checks.
          This allows preflight checks to be run on demand, which may be useful on systems that normally run them at low rate, or which do not trigger checks when the armable state might have changed.
          This command should return MAV_RESULT_ACCEPTED if it will run the checks.
          The results of the checks are usually then reported in SYS_STATUS, EVENT, or STATUSTEXT messages (this is system-specific).
          The command should return MAV_RESULT_TEMPORARILY_REJECTED if the system is already armed.
         |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    ILLUMINATOR_ON_OFF=405, /* Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the system: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). |Illuminators on (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE turns illuminators off. Values not equal to 0 or 1 are invalid.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_ILLUMINATOR_CONFIGURE=406, /* Configures illuminator settings. An illuminator is a light source that is used for lighting up dark areas external to the system: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). |Mode| 0%: Off, 100%: Max Brightness| Strobe period in seconds where 0 means strobing is not used| Strobe duty cycle where 100% means it is on constantly and 0 means strobing is not used| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    GET_HOME_POSITION=410, /* Request the home position from the vehicle.
          The vehicle will ACK the command and emit the HOME_POSITION message. |Reserved| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    INJECT_FAILURE=420, /* Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting. |The unit which is affected by the failure.| The type how the failure manifests itself.| Instance affected by failure (0 to signal all). Takes precedence over Instance bitmask (param4) when not NaN. Set to NaN to use Instance bitmask instead.| Bitmask of instances affected by the failure (bit 0 = first instance, bit 1 = second instance, etc.). Used only when Instance (param3) is NaN.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    START_RX_PAIR=500, /* Starts receiver pairing. |RC type.| RC sub type.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    GET_MESSAGE_INTERVAL=510, /* 
          Request the interval between messages for a particular MAVLink message ID.
          The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message.
         |The MAVLink message ID| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_MESSAGE_INTERVAL=511, /* Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. |The MAVLink message ID| The interval between two messages. -1: disable. 0: request default rate (which may be zero).| Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).  When used as an index ID, 0 means "all instances", "1" means the first instance in the sequence (the emitted message will have an id of 0 if message ids are 0-indexed, or 1 if index numbers start from one).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0/NaN).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0/NaN).| Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requester, 2: broadcast.|  */
    REQUEST_MESSAGE=512, /* Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL). |The MAVLink message ID of the requested message.| Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requester, 2: broadcast.|  */
    REQUEST_PROTOCOL_VERSION=519, /* Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message |Request supported protocol versions by all nodes on the network (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message |Request autopilot version (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_CAMERA_INFORMATION=521, /* Request camera information (CAMERA_INFORMATION). |Request camera capabilities (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_CAMERA_SETTINGS=522, /* Request camera settings (CAMERA_SETTINGS). |Request camera settings (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_STORAGE_INFORMATION=525, /* Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. |Storage ID (0 for all, 1 for first, 2 for second, etc.)| Request storage information (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    STORAGE_FORMAT=526, /* Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. |Storage ID (1 for first, 2 for second, etc.)| Format storage (and reset image log) (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reset Image Log (without formatting storage medium) (MAV_BOOL_TRUE). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_CAMERA_CAPTURE_STATUS=527, /* Request camera capture status (CAMERA_CAPTURE_STATUS) |Request camera capture status (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_FLIGHT_INFORMATION=528, /* Request flight information (FLIGHT_INFORMATION) |Request flight information (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Reserved (all remaining params)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    RESET_CAMERA_SETTINGS=529, /* Reset all camera settings to Factory Default |Reset all settings (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_CAMERA_MODE=530, /* Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. |Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Camera mode| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    SET_CAMERA_ZOOM=531, /* Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). |Zoom type| Zoom value. The range of valid values depend on the zoom type.| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_CAMERA_FOCUS=532, /* Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). |Focus type| Focus value| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_STORAGE_USAGE=533, /* Set that a particular storage is the preferred location for saving photos, videos, and/or other media (e.g. to set that an SD card is used for storing videos).
          There can only be one preferred save location for each particular media type: setting a media usage flag will clear/reset that same flag if set on any other storage.
          If no flag is set the system should use its default storage.
          A target system can choose to always use default storage, in which case it should ACK the command with MAV_RESULT_UNSUPPORTED.
          A target system can choose to not allow a particular storage to be set as preferred storage, in which case it should ACK the command with MAV_RESULT_DENIED. |Storage ID (1 for first, 2 for second, etc.)| Usage flags| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_CAMERA_SOURCE=534, /* Set camera source. Changes the camera's active sources on cameras with multiple image sensors. |Component Id of camera to address or 1-6 for non-MAVLink cameras, 0 for all cameras.| Primary Source| Secondary Source. If non-zero the second source will be displayed as picture-in-picture.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    JUMP_TAG=600, /* Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. |Tag.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_JUMP_TAG=601, /* Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. |Target tag to jump to.| Repeat count.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_SET_GLOBAL_ORIGIN=611, /* Sets the GNSS coordinates of the vehicle local origin (0,0,0) position.
          Vehicle should emit GPS_GLOBAL_ORIGIN irrespective of whether the origin is changed.
          This enables transform between the local coordinate frame and the global (GNSS) coordinate frame, which may be necessary when (for example) indoor and outdoor settings are connected and the MAV should move from in- to outdoor.
          This command supersedes SET_GPS_GLOBAL_ORIGIN.
          Should be sent in a COMMAND_INT (Expected frame is MAV_FRAME_GLOBAL, and this should be assumed when sent in COMMAND_LONG).
         |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Latitude| Longitude| Altitude|  */
    DO_GIMBAL_MANAGER_PITCHYAW=1000, /* Set gimbal manager pitch/yaw setpoints (low rate command). It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: only the gimbal manager will react to this command - it will be ignored by a gimbal device. Use GIMBAL_MANAGER_SET_PITCHYAW if you need to stream pitch/yaw setpoints at higher rate.  |Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).| Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).| Pitch rate (positive to pitch up).| Yaw rate (positive to yaw to the right).| Gimbal manager flags to use.| Reserved (default:0)| Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).|  */
    DO_GIMBAL_MANAGER_CONFIGURE=1001, /* Gimbal configuration to set which sysid/compid is in primary and secondary control. |Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| Reserved (default:0)| Reserved (default:0)| Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).|  */
    IMAGE_START_CAPTURE=2000, /* Start image capture sequence. CAMERA_IMAGE_CAPTURED must be emitted after each capture.

          Param1 (id) may be used to specify the target camera: 0: all cameras, 1 to 6: autopilot-connected cameras, 7-255: MAVLink camera component ID.
          It is needed in order to target specific cameras connected to the autopilot, or specific sensors in a multi-sensor camera (neither of which have a distinct MAVLink component ID).
          It is also needed to specify the target camera in missions.

          When used in a mission, an autopilot should execute the MAV_CMD for a specified local camera (param1 = 1-6), or resend it as a command if it is intended for a MAVLink camera (param1 = 7 - 255), setting the command's target_component as the param1 value (and setting param1 in the command to zero).
          If the param1 is 0 the autopilot should do both.

          When sent in a command the target MAVLink address is set using target_component.
          If addressed specifically to an autopilot: param1 should be used in the same way as it is for missions (though command should NACK with MAV_RESULT_DENIED if a specified local camera does not exist).
          If addressed to a MAVLink camera, param 1 can be used to address all cameras (0), or to separately address 1 to 7 individual sensors. Other values should be NACKed with MAV_RESULT_DENIED.
          If the command is broadcast (target_component is 0) then param 1 should be set to 0 (any other value should be NACKED with MAV_RESULT_DENIED). An autopilot would trigger any local cameras and forward the command to all channels.
         |Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).| Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.| Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence.

          Param1 (id) may be used to specify the target camera: 0: all cameras, 1 to 6: autopilot-connected cameras, 7-255: MAVLink camera component ID.
          It is needed in order to target specific cameras connected to the autopilot, or specific sensors in a multi-sensor camera (neither of which have a distinct MAVLink component ID).
          It is also needed to specify the target camera in missions.

          When used in a mission, an autopilot should execute the MAV_CMD for a specified local camera (param1 = 1-6), or resend it as a command if it is intended for a MAVLink camera (param1 = 7 - 255), setting the command's target_component as the param1 value (and setting param1 in the command to zero).
          If the param1 is 0 the autopilot should do both.

          When sent in a command the target MAVLink address is set using target_component.
          If addressed specifically to an autopilot: param1 should be used in the same way as it is for missions (though command should NACK with MAV_RESULT_DENIED if a specified local camera does not exist).
          If addressed to a MAVLink camera, param1 can be used to address all cameras (0), or to separately address 1 to 7 individual sensors. Other values should be NACKed with MAV_RESULT_DENIED.
          If the command is broadcast (target_component is 0) then param 1 should be set to 0 (any other value should be NACKED with MAV_RESULT_DENIED). An autopilot would trigger any local cameras and forward the command to all channels.
         |Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    REQUEST_CAMERA_IMAGE_CAPTURE=2002, /* Re-request a CAMERA_IMAGE_CAPTURED message. |Sequence number for missing CAMERA_IMAGE_CAPTURED message| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |Trigger enable/disable (0 for disable, 1 for start), -1 to ignore| 1 to reset the trigger sequence, -1 or 0 to ignore| 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    CAMERA_TRACK_POINT=2004, /* If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. |Point to track x value (normalized 0..1, 0 is left, 1 is right).| Point to track y value (normalized 0..1, 0 is top, 1 is bottom).| Point radius (normalized 0..1, 0 is one pixel, 1 is full image width).| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    CAMERA_TRACK_RECTANGLE=2005, /* If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. |Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).| Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).| Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).| Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:0)| Reserved (default:0)|  */
    CAMERA_STOP_TRACKING=2010, /* Stops ongoing tracking. |Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    VIDEO_START_CAPTURE=2500, /* Starts video capture (recording). |Video Stream ID (0 for all streams)| Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture (recording). |Video Stream ID (0 for all streams)| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    VIDEO_START_STREAMING=2502, /* Start video streaming |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    VIDEO_STOP_STREAMING=2503, /* Stop the given video stream |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Target camera ID. 7 to 255: MAVLink camera component id. 1 to 6 for cameras attached to the autopilot, which don't have a distinct component id. 0: all cameras. This is used to target specific autopilot-connected cameras. It is also used to target specific cameras when the MAV_CMD is used in a mission.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_VIDEO_STREAM_INFORMATION=2504, /* Request video stream information (VIDEO_STREAM_INFORMATION) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    REQUEST_VIDEO_STREAM_STATUS=2505, /* Request video stream status (VIDEO_STREAM_STATUS) |Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |Format: 0: ULog| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    AIRFRAME_CONFIGURATION=2520, /*  |Landing gear ID (default: 0, -1 for all)| Landing gear position (Down: 0, Up: 1, NaN for no change)| Reserved (default:NaN)| Reserved (default:NaN)| Reserved (default:0)| Reserved (default:0)| Reserved (default:NaN)|  */
    CONTROL_HIGH_LATENCY=2600, /* Request to start/stop transmitting over the high latency telemetry |Start transmission over high latency telemetry (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE stops transmission. Values not equal to 0 or 1 are invalid.| Empty| Empty| Empty| Empty| Empty| Empty|  */
    PANORAMA_CREATE=2800, /* Create a panorama at the current position |Viewing angle horizontal of the panorama (+- 0.5 the total angle)| Viewing angle vertical of panorama.| Speed of the horizontal rotation.| Speed of the vertical rotation.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    DO_VTOL_TRANSITION=3000, /* Request VTOL transition |The target VTOL state. For normal transitions, only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.| Force immediate transition to the specified MAV_VTOL_STATE. 1: Force immediate, 0: normal transition. Can be used, for example, to trigger an emergency "Quadchute". Caution: Can be dangerous/damage vehicle, depending on autopilot implementation of this command.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    ARM_AUTHORIZATION_REQUEST=3001, /* Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request.
		If approved the COMMAND_ACK message progress field should be set with period of time that this authorization is valid in seconds.
		If the authorization is denied COMMAND_ACK.result_param2 should be set with one of the reasons in MAV_ARM_AUTH_DENIED_REASON.
         |Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
                   |Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |Radius of desired circle in CIRCLE_MODE| User defined| User defined| User defined| Target latitude of center of circle in CIRCLE_MODE| Target longitude of center of circle in CIRCLE_MODE| Reserved (default:0)|  */
    CONDITION_GATE=4501, /* Delay mission state machine until gate has been reached. |Geometry: 0: orthogonal to path between previous and next waypoint.| Use altitude (MAV_BOOL_TRUE). A value of MAV_BOOL_FALSE ignores altitude. Values not equal to 0 or 1 are invalid.| Empty| Empty| Latitude| Longitude| Altitude|  */
    NAV_FENCE_RETURN_POINT=5000, /* Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead. |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001, /* Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
          The vertices for a polygon must be sent sequentially, each with param1 set to the total number of vertices in the polygon.
         |Polygon vertex count. This is the number of vertices in the current polygon (all vertices will have the same number).| Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_POLYGON_VERTEX_EXCLUSION=5002, /* Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
          The vertices for a polygon must be sent sequentially, each with param1 set to the total number of vertices in the polygon.
         |Polygon vertex count. This is the number of vertices in the current polygon (all vertices will have the same number).| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_CIRCLE_INCLUSION=5003, /* Circular fence area. The vehicle must stay inside this area.
         |Radius.| Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_FENCE_CIRCLE_EXCLUSION=5004, /* Circular fence area. The vehicle must stay outside this area.
         |Radius.| Reserved| Reserved| Reserved| Latitude| Longitude| Reserved|  */
    NAV_RALLY_POINT=5100, /* Rally point. You can have multiple rally points defined.
         |Reserved| Reserved| Reserved| Reserved| Latitude| Longitude| Altitude|  */
    UAVCAN_GET_NODE_INFO=5200, /* Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    DO_SET_SAFETY_SWITCH_STATE=5300, /* Change state of safety switch. |New safety switch state.| Empty.| Empty.| Empty| Empty.| Empty.| Empty.|  */
    DO_ADSB_OUT_IDENT=10001, /* Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds by the hardware per the Mode A, C, and S transponder spec. |Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)| Reserved (set to 0)|  */
    PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| Desired approach vector in compass heading. A negative value indicates the system can define the approach vector at will.| Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.| Latitude.| Longitude.| Altitude (MSL)|  */
    PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| Reserved| Reserved| Reserved| Reserved| Reserved| Reserved|  */
    WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |User defined| User defined| User defined| User defined| Latitude unscaled| Longitude unscaled| Altitude (MSL)|  */
    USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |User defined| User defined| User defined| User defined| User defined| User defined| User defined|  */
    CAN_FORWARD=32000, /* Request forwarding of CAN packets from the given CAN bus to this component via this MAVLink channel. CAN Frames are sent using CAN_FRAME and CANFD_FRAME messages |Bus number (0 to disable forwarding, 1 for first bus, 2 for 2nd bus, 3 for 3rd bus).| Empty.| Empty.| Empty.| Empty.| Empty.| Empty.|  */
    FIXED_MAG_CAL_YAW=42006, /* Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location. |Yaw of vehicle in earth frame.| CompassMask, 0 for all.| Latitude.| Longitude.| Empty.| Empty.| Empty.|  */
    DO_WINCH=42600, /* Command to operate winch. |Winch instance number.| Action to perform.| Length of line to release (negative to wind).| Release rate (negative to wind).| Empty.| Empty.| Empty.|  */
    GUIDED_CHANGE_SPEED=43000, /* Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's previous speed and the new one. |Airspeed or groundspeed.| Target Speed| Acceleration rate, 0 to take effect instantly| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    GUIDED_CHANGE_ALTITUDE=43001, /* Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous altitude and the new one. |Reserved (default:0)| Reserved (default:0)| Rate of change, toward new altitude. 0 for maximum rate change. Positive numbers only, as negative numbers will not converge on the new target alt.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Target Altitude|  */
    GUIDED_CHANGE_HEADING=43002, /* Change to target direction at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between its previous heading and the new one. |Course-over-ground or raw vehicle heading.| Target heading.| Maximum centripetal acceleration, i.e. rate of change toward new heading.| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)| Reserved (default:0)|  */
    EXTERNAL_POSITION_ESTIMATE=43003, /* Provide an external position estimate for use when dead-reckoning. This is meant to be used for occasional position resets that may be provided by a external system such as a remote pilot using landmarks over a video link. |Timestamp that this message was sent as a time in the transmitters time domain. The sender should wrap this time back to zero based on required timing accuracy for the application and the limitations of a 32 bit float. For example, wrapping at 10 hours would give approximately 1ms accuracy. Recipient must handle time wrap in any timing jitter correction applied to this field. Wrap rollover time should not be at not more than 250 seconds, which would give approximately 10 microsecond accuracy.| The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. Set to zero if not known.| estimated one standard deviation accuracy of the measurement. Set to NaN if not known.| Empty| Latitude| Longitude| Altitude, not used. Should be sent as NaN. May be supported in a future version of this message.|  */
};

//! MAV_CMD ENUM_END
constexpr auto MAV_CMD_ENUM_END = 43004;

/** @brief A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages. */
enum class MAV_DATA_STREAM : uint8_t
{
    ALL=0, /* Enable all data streams | */
    RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
    EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
    RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
    RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
    POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION_INT messages. | */
    EXTRA1=10, /* Dependent on the autopilot | */
    EXTRA2=11, /* Dependent on the autopilot | */
    EXTRA3=12, /* Dependent on the autopilot | */
};

//! MAV_DATA_STREAM ENUM_END
constexpr auto MAV_DATA_STREAM_ENUM_END = 13;

/** @brief The ROI (region of interest) for the vehicle. This can be used by the vehicle for camera/vehicle attitude alignment (see MAV_CMD_NAV_ROI). */
enum class MAV_ROI
{
    NONE=0, /* No region of interest. | */
    WPNEXT=1, /* Point toward next waypoint, with optional pitch/roll/yaw offset. | */
    WPINDEX=2, /* Point toward given waypoint. | */
    LOCATION=3, /* Point toward fixed location. | */
    TARGET=4, /* Point toward of given id. | */
};

//! MAV_ROI ENUM_END
constexpr auto MAV_ROI_ENUM_END = 5;

/** @brief Specifies the datatype of a MAVLink parameter. */
enum class MAV_PARAM_TYPE : uint8_t
{
    UINT8=1, /* 8-bit unsigned integer | */
    INT8=2, /* 8-bit signed integer | */
    UINT16=3, /* 16-bit unsigned integer | */
    INT16=4, /* 16-bit signed integer | */
    UINT32=5, /* 32-bit unsigned integer | */
    INT32=6, /* 32-bit signed integer | */
    UINT64=7, /* 64-bit unsigned integer | */
    INT64=8, /* 64-bit signed integer | */
    REAL32=9, /* 32-bit floating-point | */
    REAL64=10, /* 64-bit floating-point | */
};

//! MAV_PARAM_TYPE ENUM_END
constexpr auto MAV_PARAM_TYPE_ENUM_END = 11;

/** @brief Parameter protocol error types (see PARAM_ERROR). */
enum class MAV_PARAM_ERROR : uint8_t
{
    NO_ERROR=0, /* No error occurred (not expected in PARAM_ERROR but may be used in future implementations. | */
    DOES_NOT_EXIST=1, /* Parameter does not exist | */
    VALUE_OUT_OF_RANGE=2, /* Parameter value does not fit within accepted range | */
    PERMISSION_DENIED=3, /* Caller is not permitted to set the value of this parameter | */
    COMPONENT_NOT_FOUND=4, /* Unknown component specified | */
    READ_ONLY=5, /* Parameter is read-only | */
    TYPE_UNSUPPORTED=6, /* Parameter data type (MAV_PARAM_TYPE) is not supported by flight stack (at all) | */
    TYPE_MISMATCH=7, /* Parameter type does not match expected type | */
    READ_FAIL=8, /* Parameter exists but reading failed | */
};

//! MAV_PARAM_ERROR ENUM_END
constexpr auto MAV_PARAM_ERROR_ENUM_END = 9;

/** @brief Specifies the datatype of a MAVLink extended parameter. */
enum class MAV_PARAM_EXT_TYPE : uint8_t
{
    UINT8=1, /* 8-bit unsigned integer | */
    INT8=2, /* 8-bit signed integer | */
    UINT16=3, /* 16-bit unsigned integer | */
    INT16=4, /* 16-bit signed integer | */
    UINT32=5, /* 32-bit unsigned integer | */
    INT32=6, /* 32-bit signed integer | */
    UINT64=7, /* 64-bit unsigned integer | */
    INT64=8, /* 64-bit signed integer | */
    REAL32=9, /* 32-bit floating-point | */
    REAL64=10, /* 64-bit floating-point | */
    CUSTOM=11, /* Custom Type | */
};

//! MAV_PARAM_EXT_TYPE ENUM_END
constexpr auto MAV_PARAM_EXT_TYPE_ENUM_END = 12;

/** @brief Result from a MAVLink command (MAV_CMD) */
enum class MAV_RESULT : uint8_t
{
    ACCEPTED=0, /* Command is valid (is supported and has valid parameters), and was executed. | */
    TEMPORARILY_REJECTED=1, /* Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work. | */
    DENIED=2, /* Command is invalid; it is supported but one or more parameter values are invalid (i.e. parameter reserved, value allowed by spec but not supported by flight stack, and so on). Retrying the same command and parameters will not work. | */
    UNSUPPORTED=3, /* Command is not supported (unknown). | */
    FAILED=4, /* Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc. | */
    IN_PROGRESS=5, /* Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation. | */
    CANCELLED=6, /* Command has been cancelled (as a result of receiving a COMMAND_CANCEL message). | */
    COMMAND_LONG_ONLY=7, /* Command is only accepted when sent as a COMMAND_LONG. | */
    COMMAND_INT_ONLY=8, /* Command is only accepted when sent as a COMMAND_INT. | */
    COMMAND_UNSUPPORTED_MAV_FRAME=9, /* Command is invalid because a frame is required and the specified frame is not supported. | */
    NOT_IN_CONTROL=10, /* Command has been rejected because source system is not in control of the target system/component. | */
};

//! MAV_RESULT ENUM_END
constexpr auto MAV_RESULT_ENUM_END = 11;

/** @brief Result of mission operation (in a MISSION_ACK message). */
enum class MAV_MISSION_RESULT : uint8_t
{
    ACCEPTED=0, /* mission accepted OK | */
    ERROR=1, /* Generic error / not accepting mission commands at all right now. | */
    UNSUPPORTED_FRAME=2, /* Coordinate frame is not supported. | */
    UNSUPPORTED=3, /* Command is not supported. | */
    NO_SPACE=4, /* Mission items exceed storage space. | */
    INVALID=5, /* One of the parameters has an invalid value. | */
    INVALID_PARAM1=6, /* param1 has an invalid value. | */
    INVALID_PARAM2=7, /* param2 has an invalid value. | */
    INVALID_PARAM3=8, /* param3 has an invalid value. | */
    INVALID_PARAM4=9, /* param4 has an invalid value. | */
    INVALID_PARAM5_X=10, /* x / param5 has an invalid value. | */
    INVALID_PARAM6_Y=11, /* y / param6 has an invalid value. | */
    INVALID_PARAM7=12, /* z / param7 has an invalid value. | */
    INVALID_SEQUENCE=13, /* Mission item received out of sequence | */
    DENIED=14, /* Not accepting any mission commands from this communication partner. | */
    OPERATION_CANCELLED=15, /* Current mission operation cancelled (e.g. mission upload, mission download). | */
};

//! MAV_MISSION_RESULT ENUM_END
constexpr auto MAV_MISSION_RESULT_ENUM_END = 16;

/** @brief Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/. */
enum class MAV_SEVERITY : uint8_t
{
    EMERGENCY=0, /* System is unusable. This is a "panic" condition. | */
    ALERT=1, /* Action should be taken immediately. Indicates error in non-critical systems. | */
    CRITICAL=2, /* Action must be taken immediately. Indicates failure in a primary system. | */
    ERROR=3, /* Indicates an error in secondary/redundant systems. | */
    WARNING=4, /* Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. | */
    NOTICE=5, /* An unusual event has occurred, though not an error condition. This should be investigated for the root cause. | */
    INFO=6, /* Normal operational messages. Useful for logging. No action is required for these messages. | */
    DEBUG=7, /* Useful non-operational messages that can assist in debugging. These should not occur during normal operation. | */
};

//! MAV_SEVERITY ENUM_END
constexpr auto MAV_SEVERITY_ENUM_END = 8;

/** @brief Power supply status flags (bitmask) */
enum class MAV_POWER_STATUS : uint16_t
{
    BRICK_VALID=1, /* main brick power supply valid | */
    SERVO_VALID=2, /* main servo power supply valid for FMU | */
    USB_CONNECTED=4, /* USB power is connected | */
    PERIPH_OVERCURRENT=8, /* peripheral supply is in over-current state | */
    PERIPH_HIPOWER_OVERCURRENT=16, /* hi-power peripheral supply is in over-current state | */
    CHANGED=32, /* Power status has changed since boot | */
};

//! MAV_POWER_STATUS ENUM_END
constexpr auto MAV_POWER_STATUS_ENUM_END = 33;

/** @brief SERIAL_CONTROL device types */
enum class SERIAL_CONTROL_DEV : uint8_t
{
    TELEM1=0, /* First telemetry port | */
    TELEM2=1, /* Second telemetry port | */
    GPS1=2, /* First GPS port | */
    GPS2=3, /* Second GPS port | */
    TELEM3=4, /* Third telemetry port | */
    TELEM4=5, /* Fourth telemetry port | */
    SHELL=10, /* system shell | */
    ESC0=20, /* Electronic Speed Controller 0 | */
    ESC1=21, /* Electronic Speed Controller 1 | */
    ESC2=22, /* Electronic Speed Controller 2 | */
    ESC3=23, /* Electronic Speed Controller 3 | */
    ESC4=24, /* Electronic Speed Controller 4 | */
    ESC5=25, /* Electronic Speed Controller 5 | */
    ESC6=26, /* Electronic Speed Controller 6 | */
    ESC7=27, /* Electronic Speed Controller 7 | */
    SERIAL0=100, /* SERIAL0 | */
    SERIAL1=101, /* SERIAL1 | */
    SERIAL2=102, /* SERIAL2 | */
    SERIAL3=103, /* SERIAL3 | */
    SERIAL4=104, /* SERIAL4 | */
    SERIAL5=105, /* SERIAL5 | */
    SERIAL6=106, /* SERIAL6 | */
    SERIAL7=107, /* SERIAL7 | */
    SERIAL8=108, /* SERIAL8 | */
    SERIAL9=109, /* SERIAL9 | */
};

//! SERIAL_CONTROL_DEV ENUM_END
constexpr auto SERIAL_CONTROL_DEV_ENUM_END = 110;

/** @brief SERIAL_CONTROL flags (bitmask) */
enum class SERIAL_CONTROL_FLAG : uint8_t
{
    REPLY=1, /* Set if this is a reply | */
    RESPOND=2, /* Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message | */
    EXCLUSIVE=4, /* Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set | */
    BLOCKING=8, /* Block on writes to the serial port | */
    MULTI=16, /* Send multiple replies until port is drained | */
};

//! SERIAL_CONTROL_FLAG ENUM_END
constexpr auto SERIAL_CONTROL_FLAG_ENUM_END = 17;

/** @brief Enumeration of distance sensor types */
enum class MAV_DISTANCE_SENSOR : uint8_t
{
    LASER=0, /* Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units | */
    ULTRASOUND=1, /* Ultrasound rangefinder, e.g. MaxBotix units | */
    INFRARED=2, /* Infrared rangefinder, e.g. Sharp units | */
    RADAR=3, /* Radar type, e.g. uLanding units | */
    UNKNOWN=4, /* Broken or unknown type, e.g. analog units | */
};

//! MAV_DISTANCE_SENSOR ENUM_END
constexpr auto MAV_DISTANCE_SENSOR_ENUM_END = 5;

/** @brief Enumeration of sensor orientation, according to its rotations */
enum class MAV_SENSOR_ORIENTATION : uint8_t
{
    ROTATION_NONE=0, /* Roll: 0, Pitch: 0, Yaw: 0 | */
    ROTATION_YAW_45=1, /* Roll: 0, Pitch: 0, Yaw: 45 | */
    ROTATION_YAW_90=2, /* Roll: 0, Pitch: 0, Yaw: 90 | */
    ROTATION_YAW_135=3, /* Roll: 0, Pitch: 0, Yaw: 135 | */
    ROTATION_YAW_180=4, /* Roll: 0, Pitch: 0, Yaw: 180 | */
    ROTATION_YAW_225=5, /* Roll: 0, Pitch: 0, Yaw: 225 | */
    ROTATION_YAW_270=6, /* Roll: 0, Pitch: 0, Yaw: 270 | */
    ROTATION_YAW_315=7, /* Roll: 0, Pitch: 0, Yaw: 315 | */
    ROTATION_ROLL_180=8, /* Roll: 180, Pitch: 0, Yaw: 0 | */
    ROTATION_ROLL_180_YAW_45=9, /* Roll: 180, Pitch: 0, Yaw: 45 | */
    ROTATION_ROLL_180_YAW_90=10, /* Roll: 180, Pitch: 0, Yaw: 90 | */
    ROTATION_ROLL_180_YAW_135=11, /* Roll: 180, Pitch: 0, Yaw: 135 | */
    ROTATION_PITCH_180=12, /* Roll: 0, Pitch: 180, Yaw: 0 | */
    ROTATION_ROLL_180_YAW_225=13, /* Roll: 180, Pitch: 0, Yaw: 225 | */
    ROTATION_ROLL_180_YAW_270=14, /* Roll: 180, Pitch: 0, Yaw: 270 | */
    ROTATION_ROLL_180_YAW_315=15, /* Roll: 180, Pitch: 0, Yaw: 315 | */
    ROTATION_ROLL_90=16, /* Roll: 90, Pitch: 0, Yaw: 0 | */
    ROTATION_ROLL_90_YAW_45=17, /* Roll: 90, Pitch: 0, Yaw: 45 | */
    ROTATION_ROLL_90_YAW_90=18, /* Roll: 90, Pitch: 0, Yaw: 90 | */
    ROTATION_ROLL_90_YAW_135=19, /* Roll: 90, Pitch: 0, Yaw: 135 | */
    ROTATION_ROLL_270=20, /* Roll: 270, Pitch: 0, Yaw: 0 | */
    ROTATION_ROLL_270_YAW_45=21, /* Roll: 270, Pitch: 0, Yaw: 45 | */
    ROTATION_ROLL_270_YAW_90=22, /* Roll: 270, Pitch: 0, Yaw: 90 | */
    ROTATION_ROLL_270_YAW_135=23, /* Roll: 270, Pitch: 0, Yaw: 135 | */
    ROTATION_PITCH_90=24, /* Roll: 0, Pitch: 90, Yaw: 0 | */
    ROTATION_PITCH_270=25, /* Roll: 0, Pitch: 270, Yaw: 0 | */
    ROTATION_PITCH_180_YAW_90=26, /* Roll: 0, Pitch: 180, Yaw: 90 | */
    ROTATION_PITCH_180_YAW_270=27, /* Roll: 0, Pitch: 180, Yaw: 270 | */
    ROTATION_ROLL_90_PITCH_90=28, /* Roll: 90, Pitch: 90, Yaw: 0 | */
    ROTATION_ROLL_180_PITCH_90=29, /* Roll: 180, Pitch: 90, Yaw: 0 | */
    ROTATION_ROLL_270_PITCH_90=30, /* Roll: 270, Pitch: 90, Yaw: 0 | */
    ROTATION_ROLL_90_PITCH_180=31, /* Roll: 90, Pitch: 180, Yaw: 0 | */
    ROTATION_ROLL_270_PITCH_180=32, /* Roll: 270, Pitch: 180, Yaw: 0 | */
    ROTATION_ROLL_90_PITCH_270=33, /* Roll: 90, Pitch: 270, Yaw: 0 | */
    ROTATION_ROLL_180_PITCH_270=34, /* Roll: 180, Pitch: 270, Yaw: 0 | */
    ROTATION_ROLL_270_PITCH_270=35, /* Roll: 270, Pitch: 270, Yaw: 0 | */
    ROTATION_ROLL_90_PITCH_180_YAW_90=36, /* Roll: 90, Pitch: 180, Yaw: 90 | */
    ROTATION_ROLL_90_YAW_270=37, /* Roll: 90, Pitch: 0, Yaw: 270 | */
    ROTATION_ROLL_90_PITCH_68_YAW_293=38, /* Roll: 90, Pitch: 68, Yaw: 293 | */
    ROTATION_PITCH_315=39, /* Pitch: 315 | */
    ROTATION_ROLL_90_PITCH_315=40, /* Roll: 90, Pitch: 315 | */
    ROTATION_CUSTOM=100, /* Custom orientation | */
};

//! MAV_SENSOR_ORIENTATION ENUM_END
constexpr auto MAV_SENSOR_ORIENTATION_ENUM_END = 101;

/** @brief Type of mission items being requested/sent in mission protocol. */
enum class MAV_MISSION_TYPE : uint8_t
{
    MISSION=0, /* Items are mission commands for main mission. | */
    FENCE=1, /* Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items. | */
    RALLY=2, /* Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items. | */
    ALL=255, /* Only used in MISSION_CLEAR_ALL to clear all mission types. | */
};

//! MAV_MISSION_TYPE ENUM_END
constexpr auto MAV_MISSION_TYPE_ENUM_END = 256;

/** @brief Enumeration of estimator types */
enum class MAV_ESTIMATOR_TYPE : uint8_t
{
    UNKNOWN=0, /* Unknown type of the estimator. | */
    NAIVE=1, /* This is a naive estimator without any real covariance feedback. | */
    VISION=2, /* Computer vision based estimate. Might be up to scale. | */
    VIO=3, /* Visual-inertial estimate. | */
    GPS=4, /* Plain GPS estimate. | */
    GPS_INS=5, /* Estimator integrating GPS and inertial sensing. | */
    MOCAP=6, /* Estimate from external motion capturing system. | */
    LIDAR=7, /* Estimator based on lidar sensor input. | */
    AUTOPILOT=8, /* Estimator on autopilot. | */
};

//! MAV_ESTIMATOR_TYPE ENUM_END
constexpr auto MAV_ESTIMATOR_TYPE_ENUM_END = 9;

/** @brief Enumeration of battery types */
enum class MAV_BATTERY_TYPE : uint8_t
{
    UNKNOWN=0, /* Not specified. | */
    LIPO=1, /* Lithium polymer battery | */
    LIFE=2, /* Lithium-iron-phosphate battery | */
    LION=3, /* Lithium-ION battery | */
    NIMH=4, /* Nickel metal hydride battery | */
};

//! MAV_BATTERY_TYPE ENUM_END
constexpr auto MAV_BATTERY_TYPE_ENUM_END = 5;

/** @brief Enumeration of battery functions */
enum class MAV_BATTERY_FUNCTION : uint8_t
{
    UNKNOWN=0, /* Battery function is unknown | */
    ALL=1, /* Battery supports all flight systems | */
    PROPULSION=2, /* Battery for the propulsion system | */
    AVIONICS=3, /* Avionics battery | */
    PAYLOAD=4, /* Payload battery | */
};

//! MAV_BATTERY_FUNCTION ENUM_END
constexpr auto MAV_BATTERY_FUNCTION_ENUM_END = 5;

/** @brief Enumeration for battery charge states. */
enum class MAV_BATTERY_CHARGE_STATE : uint8_t
{
    UNDEFINED=0, /* Low battery state is not provided | */
    OK=1, /* Battery is not in low state. Normal operation. | */
    LOW=2, /* Battery state is low, warn and monitor close. | */
    CRITICAL=3, /* Battery state is critical, return or abort immediately. | */
    EMERGENCY=4, /* Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage. | */
    FAILED=5, /* Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT. | */
    UNHEALTHY=6, /* Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT. | */
    CHARGING=7, /* Battery is charging. | */
};

//! MAV_BATTERY_CHARGE_STATE ENUM_END
constexpr auto MAV_BATTERY_CHARGE_STATE_ENUM_END = 8;

/** @brief Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight. */
enum class MAV_BATTERY_MODE : uint8_t
{
    UNKNOWN=0, /* Battery mode not supported/unknown battery mode/normal operation. | */
    AUTO_DISCHARGING=1, /* Battery is auto discharging (towards storage level). | */
    HOT_SWAP=2, /* Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits). | */
};

//! MAV_BATTERY_MODE ENUM_END
constexpr auto MAV_BATTERY_MODE_ENUM_END = 3;

/** @brief Smart battery supply status/fault flags (bitmask) for health indication. The battery must also report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these are set. */
enum class MAV_BATTERY_FAULT : uint32_t
{
    DEEP_DISCHARGE=1, /* Battery has deep discharged. | */
    SPIKES=2, /* Voltage spikes. | */
    CELL_FAIL=4, /* One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILED (and should not be used). | */
    OVER_CURRENT=8, /* Over-current fault. | */
    OVER_TEMPERATURE=16, /* Over-temperature fault. | */
    UNDER_TEMPERATURE=32, /* Under-temperature fault. | */
    INCOMPATIBLE_VOLTAGE=64, /* Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage). | */
    INCOMPATIBLE_FIRMWARE=128, /* Battery firmware is not compatible with current autopilot firmware. | */
    BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION=256, /* Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s). | */
};

//! MAV_BATTERY_FAULT ENUM_END
constexpr auto MAV_BATTERY_FAULT_ENUM_END = 257;

/** @brief Fuel types for use in FUEL_TYPE. Fuel types specify the units for the maximum, available and consumed fuel, and for the flow rates. */
enum class MAV_FUEL_TYPE : uint32_t
{
    UNKNOWN=0, /* Not specified. Fuel levels are normalized (i.e. maximum is 1, and other levels are relative to 1). | */
    LIQUID=1, /* A generic liquid fuel. Fuel levels are in millilitres (ml). Fuel rates are in millilitres/second. | */
    GAS=2, /* A gas tank. Fuel levels are in kilo-Pascal (kPa), and flow rates are in milliliters per second (ml/s). | */
};

//! MAV_FUEL_TYPE ENUM_END
constexpr auto MAV_FUEL_TYPE_ENUM_END = 3;

/** @brief Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that FAULTS are conditions that cause the generator to fail. Warnings are conditions that require attention before the next use (they indicate the system is not operating properly). */
enum class MAV_GENERATOR_STATUS_FLAG : uint64_t
{
    OFF=1, /* Generator is off. | */
    READY=2, /* Generator is ready to start generating power. | */
    GENERATING=4, /* Generator is generating power. | */
    CHARGING=8, /* Generator is charging the batteries (generating enough power to charge and provide the load). | */
    REDUCED_POWER=16, /* Generator is operating at a reduced maximum power. | */
    MAXPOWER=32, /* Generator is providing the maximum output. | */
    OVERTEMP_WARNING=64, /* Generator is near the maximum operating temperature, cooling is insufficient. | */
    OVERTEMP_FAULT=128, /* Generator hit the maximum operating temperature and shutdown. | */
    ELECTRONICS_OVERTEMP_WARNING=256, /* Power electronics are near the maximum operating temperature, cooling is insufficient. | */
    ELECTRONICS_OVERTEMP_FAULT=512, /* Power electronics hit the maximum operating temperature and shutdown. | */
    ELECTRONICS_FAULT=1024, /* Power electronics experienced a fault and shutdown. | */
    POWERSOURCE_FAULT=2048, /* The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening. | */
    COMMUNICATION_WARNING=4096, /* Generator controller having communication problems. | */
    COOLING_WARNING=8192, /* Power electronic or generator cooling system error. | */
    POWER_RAIL_FAULT=16384, /* Generator controller power rail experienced a fault. | */
    OVERCURRENT_FAULT=32768, /* Generator controller exceeded the overcurrent threshold and shutdown to prevent damage. | */
    BATTERY_OVERCHARGE_CURRENT_FAULT=65536, /* Generator controller detected a high current going into the batteries and shutdown to prevent battery damage. | */
    OVERVOLTAGE_FAULT=131072, /* Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating. | */
    BATTERY_UNDERVOLT_FAULT=262144, /* Batteries are under voltage (generator will not start). | */
    START_INHIBITED=524288, /* Generator start is inhibited by e.g. a safety switch. | */
    MAINTENANCE_REQUIRED=1048576, /* Generator requires maintenance. | */
    WARMING_UP=2097152, /* Generator is not ready to generate yet. | */
    IDLE=4194304, /* Generator is idle. | */
};

//! MAV_GENERATOR_STATUS_FLAG ENUM_END
constexpr auto MAV_GENERATOR_STATUS_FLAG_ENUM_END = 4194305;

/** @brief Enumeration of VTOL states */
enum class MAV_VTOL_STATE : uint8_t
{
    UNDEFINED=0, /* MAV is not configured as VTOL | */
    TRANSITION_TO_FW=1, /* VTOL is in transition from multicopter to fixed-wing | */
    TRANSITION_TO_MC=2, /* VTOL is in transition from fixed-wing to multicopter | */
    MC=3, /* VTOL is in multicopter state | */
    FW=4, /* VTOL is in fixed-wing state | */
};

//! MAV_VTOL_STATE ENUM_END
constexpr auto MAV_VTOL_STATE_ENUM_END = 5;

/** @brief Enumeration of landed detector states */
enum class MAV_LANDED_STATE : uint8_t
{
    UNDEFINED=0, /* MAV landed state is unknown | */
    ON_GROUND=1, /* MAV is landed (on ground) | */
    IN_AIR=2, /* MAV is in air | */
    TAKEOFF=3, /* MAV currently taking off | */
    LANDING=4, /* MAV currently landing | */
};

//! MAV_LANDED_STATE ENUM_END
constexpr auto MAV_LANDED_STATE_ENUM_END = 5;

/** @brief Enumeration of the ADSB altimeter types */
enum class ADSB_ALTITUDE_TYPE : uint8_t
{
    PRESSURE_QNH=0, /* Altitude reported from a Baro source using QNH reference | */
    GEOMETRIC=1, /* Altitude reported from a GNSS source | */
};

//! ADSB_ALTITUDE_TYPE ENUM_END
constexpr auto ADSB_ALTITUDE_TYPE_ENUM_END = 2;

/** @brief ADSB classification for the type of vehicle emitting the transponder signal */
enum class ADSB_EMITTER_TYPE : uint8_t
{
    NO_INFO=0, /*  | */
    LIGHT=1, /*  | */
    SMALL=2, /*  | */
    LARGE=3, /*  | */
    HIGH_VORTEX_LARGE=4, /*  | */
    HEAVY=5, /*  | */
    HIGHLY_MANUV=6, /*  | */
    ROTOCRAFT=7, /*  | */
    UNASSIGNED=8, /*  | */
    GLIDER=9, /*  | */
    LIGHTER_AIR=10, /*  | */
    PARACHUTE=11, /*  | */
    ULTRA_LIGHT=12, /*  | */
    UNASSIGNED2=13, /*  | */
    UAV=14, /*  | */
    SPACE=15, /*  | */
    UNASSGINED3=16, /*  | */
    EMERGENCY_SURFACE=17, /*  | */
    SERVICE_SURFACE=18, /*  | */
    POINT_OBSTACLE=19, /*  | */
};

//! ADSB_EMITTER_TYPE ENUM_END
constexpr auto ADSB_EMITTER_TYPE_ENUM_END = 20;

/** @brief These flags indicate status such as data validity of each data source. Set = data valid */
enum class ADSB_FLAGS : uint16_t
{
    VALID_COORDS=1, /*  | */
    VALID_ALTITUDE=2, /*  | */
    VALID_HEADING=4, /*  | */
    VALID_VELOCITY=8, /*  | */
    VALID_CALLSIGN=16, /*  | */
    VALID_SQUAWK=32, /*  | */
    SIMULATED=64, /*  | */
    VERTICAL_VELOCITY_VALID=128, /*  | */
    BARO_VALID=256, /*  | */
    SOURCE_UAT=32768, /*  | */
};

//! ADSB_FLAGS ENUM_END
constexpr auto ADSB_FLAGS_ENUM_END = 32769;

/** @brief Bitmap of options for the MAV_CMD_DO_REPOSITION */
enum class MAV_DO_REPOSITION_FLAGS
{
    CHANGE_MODE=1, /* The aircraft should immediately transition into guided. This should not be set for follow me applications | */
    RELATIVE_YAW=2, /* Yaw relative to the vehicle current heading (if not set, relative to North). | */
};

//! MAV_DO_REPOSITION_FLAGS ENUM_END
constexpr auto MAV_DO_REPOSITION_FLAGS_ENUM_END = 3;

/** @brief Speed setpoint types used in MAV_CMD_DO_CHANGE_SPEED */
enum class SPEED_TYPE
{
    AIRSPEED=0, /* Airspeed | */
    GROUNDSPEED=1, /* Groundspeed | */
    CLIMB_SPEED=2, /* Climb speed | */
    DESCENT_SPEED=3, /* Descent speed | */
};

//! SPEED_TYPE ENUM_END
constexpr auto SPEED_TYPE_ENUM_END = 4;

/** @brief Heading setpoint types used in MAV_CMD_GUIDED_CHANGE_HEADING */
enum class HEADING_TYPE
{
    COURSE_OVER_GROUND=0, /* Course over ground. | */
    HEADING=1, /* Raw vehicle heading. | */
    DEFAULT=2, /* Default heading. | */
};

//! HEADING_TYPE ENUM_END
constexpr auto HEADING_TYPE_ENUM_END = 3;

/** @brief Flags in ESTIMATOR_STATUS message */
enum class ESTIMATOR_STATUS_FLAGS : uint16_t
{
    ATTITUDE=1, /* True if the attitude estimate is good | */
    VELOCITY_HORIZ=2, /* True if the horizontal velocity estimate is good | */
    VELOCITY_VERT=4, /* True if the  vertical velocity estimate is good | */
    POS_HORIZ_REL=8, /* True if the horizontal position (relative) estimate is good | */
    POS_HORIZ_ABS=16, /* True if the horizontal position (absolute) estimate is good | */
    POS_VERT_ABS=32, /* True if the vertical position (absolute) estimate is good | */
    POS_VERT_AGL=64, /* True if the vertical position (above ground) estimate is good | */
    CONST_POS_MODE=128, /* True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow) | */
    PRED_POS_HORIZ_REL=256, /* True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate | */
    PRED_POS_HORIZ_ABS=512, /* True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate | */
    GPS_GLITCH=1024, /* True if the EKF has detected a GPS glitch | */
    ACCEL_ERROR=2048, /* True if the EKF has detected bad accelerometer data | */
};

//! ESTIMATOR_STATUS_FLAGS ENUM_END
constexpr auto ESTIMATOR_STATUS_FLAGS_ENUM_END = 2049;

/** @brief Sequence that motors are tested when using MAV_CMD_DO_MOTOR_TEST. */
enum class MOTOR_TEST_ORDER
{
    DEFAULT=0, /* Default autopilot motor test method. | */
    SEQUENCE=1, /* Motor numbers are specified as their index in a predefined vehicle-specific sequence. | */
    BOARD=2, /* Motor numbers are specified as the output as labeled on the board. | */
};

//! MOTOR_TEST_ORDER ENUM_END
constexpr auto MOTOR_TEST_ORDER_ENUM_END = 3;

/** @brief Defines how throttle value is represented in MAV_CMD_DO_MOTOR_TEST. */
enum class MOTOR_TEST_THROTTLE_TYPE
{
    PERCENT=0, /* Throttle as a percentage (0 ~ 100) | */
    PWM=1, /* Throttle as an absolute PWM value (normally in range of 1000~2000). | */
    PILOT=2, /* Throttle pass-through from pilot's transmitter. | */
    COMPASS_CAL=3, /* Per-motor compass calibration test. | */
};

//! MOTOR_TEST_THROTTLE_TYPE ENUM_END
constexpr auto MOTOR_TEST_THROTTLE_TYPE_ENUM_END = 4;

/** @brief  */
enum class GPS_INPUT_IGNORE_FLAGS : uint16_t
{
    FLAG_ALT=1, /* ignore altitude field | */
    FLAG_HDOP=2, /* ignore hdop field | */
    FLAG_VDOP=4, /* ignore vdop field | */
    FLAG_VEL_HORIZ=8, /* ignore horizontal velocity field (vn and ve) | */
    FLAG_VEL_VERT=16, /* ignore vertical velocity field (vd) | */
    FLAG_SPEED_ACCURACY=32, /* ignore speed accuracy field | */
    FLAG_HORIZONTAL_ACCURACY=64, /* ignore horizontal accuracy field | */
    FLAG_VERTICAL_ACCURACY=128, /* ignore vertical accuracy field | */
};

//! GPS_INPUT_IGNORE_FLAGS ENUM_END
constexpr auto GPS_INPUT_IGNORE_FLAGS_ENUM_END = 129;

/** @brief Possible actions an aircraft can take to avoid a collision. */
enum class MAV_COLLISION_ACTION : uint8_t
{
    NONE=0, /* Ignore any potential collisions | */
    REPORT=1, /* Report potential collision | */
    ASCEND_OR_DESCEND=2, /* Ascend or Descend to avoid threat | */
    MOVE_HORIZONTALLY=3, /* Move horizontally to avoid threat | */
    MOVE_PERPENDICULAR=4, /* Aircraft to move perpendicular to the collision's velocity vector | */
    RTL=5, /* Aircraft to fly directly back to its launch point | */
    HOVER=6, /* Aircraft to stop in place | */
};

//! MAV_COLLISION_ACTION ENUM_END
constexpr auto MAV_COLLISION_ACTION_ENUM_END = 7;

/** @brief Aircraft-rated danger from this threat. */
enum class MAV_COLLISION_THREAT_LEVEL : uint8_t
{
    NONE=0, /* Not a threat | */
    LOW=1, /* Craft is mildly concerned about this threat | */
    HIGH=2, /* Craft is panicking, and may take actions to avoid threat | */
};

//! MAV_COLLISION_THREAT_LEVEL ENUM_END
constexpr auto MAV_COLLISION_THREAT_LEVEL_ENUM_END = 3;

/** @brief Source of information about this collision. */
enum class MAV_COLLISION_SRC : uint8_t
{
    ADSB=0, /* ID field references ADSB_VEHICLE packets | */
    MAVLINK_GPS_GLOBAL_INT=1, /* ID field references MAVLink SRC ID | */
};

//! MAV_COLLISION_SRC ENUM_END
constexpr auto MAV_COLLISION_SRC_ENUM_END = 2;

/** @brief Type of GPS fix */
enum class GPS_FIX_TYPE : uint8_t
{
    NO_GPS=0, /* No GPS connected | */
    NO_FIX=1, /* No position information, GPS is connected | */
    TYPE_2D_FIX=2, /* 2D position | */
    TYPE_3D_FIX=3, /* 3D position | */
    DGPS=4, /* DGPS/SBAS aided 3D position | */
    RTK_FLOAT=5, /* RTK float, 3D position | */
    RTK_FIXED=6, /* RTK Fixed, 3D position | */
    STATIC=7, /* Static fixed, typically used for base stations | */
    PPP=8, /* PPP, 3D position. | */
};

//! GPS_FIX_TYPE ENUM_END
constexpr auto GPS_FIX_TYPE_ENUM_END = 9;

/** @brief RTK GPS baseline coordinate system, used for RTK corrections */
enum class RTK_BASELINE_COORDINATE_SYSTEM : uint8_t
{
    ECEF=0, /* Earth-centered, Earth-fixed | */
    NED=1, /* RTK basestation centered, north, east, down | */
};

//! RTK_BASELINE_COORDINATE_SYSTEM ENUM_END
constexpr auto RTK_BASELINE_COORDINATE_SYSTEM_ENUM_END = 2;

/** @brief Type of landing target */
enum class LANDING_TARGET_TYPE : uint8_t
{
    LIGHT_BEACON=0, /* Landing target signaled by light beacon (ex: IR-LOCK) | */
    RADIO_BEACON=1, /* Landing target signaled by radio beacon (ex: ILS, NDB) | */
    VISION_FIDUCIAL=2, /* Landing target represented by a fiducial marker (ex: ARTag) | */
    VISION_OTHER=3, /* Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square) | */
};

//! LANDING_TARGET_TYPE ENUM_END
constexpr auto LANDING_TARGET_TYPE_ENUM_END = 4;

/** @brief Direction of VTOL transition */
enum class VTOL_TRANSITION_HEADING
{
    VEHICLE_DEFAULT=0, /* Respect the heading configuration of the vehicle. | */
    NEXT_WAYPOINT=1, /* Use the heading pointing towards the next waypoint. | */
    TAKEOFF=2, /* Use the heading on takeoff (while sitting on the ground). | */
    SPECIFIED=3, /* Use the specified heading in parameter 4. | */
    ANY=4, /* Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active). | */
};

//! VTOL_TRANSITION_HEADING ENUM_END
constexpr auto VTOL_TRANSITION_HEADING_ENUM_END = 5;

/** @brief Camera capability flags (Bitmap) */
enum class CAMERA_CAP_FLAGS : uint32_t
{
    CAPTURE_VIDEO=1, /* Camera is able to record video | */
    CAPTURE_IMAGE=2, /* Camera is able to capture images | */
    HAS_MODES=4, /* Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE) | */
    CAN_CAPTURE_IMAGE_IN_VIDEO_MODE=8, /* Camera can capture images while in video mode | */
    CAN_CAPTURE_VIDEO_IN_IMAGE_MODE=16, /* Camera can capture videos while in Photo/Image mode | */
    HAS_IMAGE_SURVEY_MODE=32, /* Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE) | */
    HAS_BASIC_ZOOM=64, /* Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM) | */
    HAS_BASIC_FOCUS=128, /* Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS) | */
    HAS_VIDEO_STREAM=256, /* Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info) | */
    HAS_TRACKING_POINT=512, /* Camera supports tracking of a point on the camera view. | */
    HAS_TRACKING_RECTANGLE=1024, /* Camera supports tracking of a selection rectangle on the camera view. | */
    HAS_TRACKING_GEO_STATUS=2048, /* Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS). | */
    HAS_THERMAL_RANGE=4096, /* Camera supports absolute thermal range (request CAMERA_THERMAL_RANGE with MAV_CMD_REQUEST_MESSAGE). | */
    HAS_MTI=8192, /* Camera supports Moving Target Indicators (MTI) on the camera view (using MAV_CMD_CAMERA_START_MTI). | */
};

//! CAMERA_CAP_FLAGS ENUM_END
constexpr auto CAMERA_CAP_FLAGS_ENUM_END = 8193;

/** @brief Stream status flags (Bitmap) */
enum class VIDEO_STREAM_STATUS_FLAGS : uint16_t
{
    RUNNING=1, /* Stream is active (running) | */
    THERMAL=2, /* Stream is thermal imaging | */
    THERMAL_RANGE_ENABLED=4, /* Stream can report absolute thermal range (see CAMERA_THERMAL_RANGE). | */
};

//! VIDEO_STREAM_STATUS_FLAGS ENUM_END
constexpr auto VIDEO_STREAM_STATUS_FLAGS_ENUM_END = 5;

/** @brief Video stream types */
enum class VIDEO_STREAM_TYPE : uint8_t
{
    RTSP=0, /* Stream is RTSP | */
    RTPUDP=1, /* Stream is RTP UDP (URI gives the port number) | */
    TCP_MPEG=2, /* Stream is MPEG on TCP | */
    MPEG_TS=3, /* Stream is MPEG TS (URI gives the port number) | */
    WHEP=4, /* Stream is WHEP (WebRTC-HTTP Egress Protocol) | */
};

//! VIDEO_STREAM_TYPE ENUM_END
constexpr auto VIDEO_STREAM_TYPE_ENUM_END = 5;

/** @brief Video stream encodings */
enum class VIDEO_STREAM_ENCODING : uint8_t
{
    UNKNOWN=0, /* Stream encoding is unknown | */
    H264=1, /* Stream encoding is H.264 | */
    H265=2, /* Stream encoding is H.265 | */
};

//! VIDEO_STREAM_ENCODING ENUM_END
constexpr auto VIDEO_STREAM_ENCODING_ENUM_END = 3;

/** @brief Camera tracking status flags */
enum class CAMERA_TRACKING_STATUS_FLAGS : uint8_t
{
    IDLE=0, /* Camera is not tracking | */
    ACTIVE=1, /* Camera is tracking | */
    ERROR=2, /* Camera tracking in error state | */
    MTI=4, /* Camera Moving Target Indicators (MTI) are active | */
    COASTING=8, /* Camera tracking target is obscured and is being predicted | */
};

//! CAMERA_TRACKING_STATUS_FLAGS ENUM_END
constexpr auto CAMERA_TRACKING_STATUS_FLAGS_ENUM_END = 9;

/** @brief Camera tracking modes */
enum class CAMERA_TRACKING_MODE : uint8_t
{
    NONE=0, /* Not tracking | */
    POINT=1, /* Target is a point | */
    RECTANGLE=2, /* Target is a rectangle | */
};

//! CAMERA_TRACKING_MODE ENUM_END
constexpr auto CAMERA_TRACKING_MODE_ENUM_END = 3;

/** @brief Camera tracking target data (shows where tracked target is within image) */
enum class CAMERA_TRACKING_TARGET_DATA : uint8_t
{
    EMBEDDED=1, /* Target data embedded in image data (proprietary) | */
    RENDERED=2, /* Target data rendered in image | */
    IN_STATUS=4, /* Target data within status message (Point or Rectangle) | */
};

//! CAMERA_TRACKING_TARGET_DATA ENUM_END
constexpr auto CAMERA_TRACKING_TARGET_DATA_ENUM_END = 5;

/** @brief Zoom types for MAV_CMD_SET_CAMERA_ZOOM */
enum class CAMERA_ZOOM_TYPE
{
    ZOOM_TYPE_STEP=0, /* Zoom one step increment (-1 for wide, 1 for tele) | */
    ZOOM_TYPE_CONTINUOUS=1, /* Continuous normalized zoom in/out rate until stopped. Range -1..1, negative: wide, positive: narrow/tele, 0 to stop zooming. Other values should be clipped to the range. | */
    ZOOM_TYPE_RANGE=2, /* Zoom value as proportion of full camera range (a percentage value between 0.0 and 100.0) | */
    ZOOM_TYPE_FOCAL_LENGTH=3, /* Zoom value/variable focal length in millimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera) | */
    ZOOM_TYPE_HORIZONTAL_FOV=4, /* Zoom value as horizontal field of view in degrees. | */
};

//! CAMERA_ZOOM_TYPE ENUM_END
constexpr auto CAMERA_ZOOM_TYPE_ENUM_END = 5;

/** @brief Focus types for MAV_CMD_SET_CAMERA_FOCUS */
enum class SET_FOCUS_TYPE
{
    FOCUS_TYPE_STEP=0, /* Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity). | */
    FOCUS_TYPE_CONTINUOUS=1, /* Continuous normalized focus in/out rate until stopped. Range -1..1, negative: in, positive: out towards infinity, 0 to stop focusing. Other values should be clipped to the range. | */
    FOCUS_TYPE_RANGE=2, /* Focus value as proportion of full camera focus range (a value between 0.0 and 100.0) | */
    FOCUS_TYPE_METERS=3, /* Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera). | */
    FOCUS_TYPE_AUTO=4, /* Focus automatically. | */
    FOCUS_TYPE_AUTO_SINGLE=5, /* Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S. | */
    FOCUS_TYPE_AUTO_CONTINUOUS=6, /* Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C. | */
};

//! SET_FOCUS_TYPE ENUM_END
constexpr auto SET_FOCUS_TYPE_ENUM_END = 7;

/** @brief Camera sources for MAV_CMD_SET_CAMERA_SOURCE */
enum class CAMERA_SOURCE
{
    DEFAULT=0, /* Default camera source. | */
    RGB=1, /* RGB camera source. | */
    IR=2, /* IR camera source. | */
    NDVI=3, /* NDVI camera source. | */
};

//! CAMERA_SOURCE ENUM_END
constexpr auto CAMERA_SOURCE_ENUM_END = 4;

/** @brief Result from PARAM_EXT_SET message. */
enum class PARAM_ACK : uint8_t
{
    ACCEPTED=0, /* Parameter value ACCEPTED and SET | */
    VALUE_UNSUPPORTED=1, /* Parameter value UNKNOWN/UNSUPPORTED | */
    FAILED=2, /* Parameter failed to set | */
    IN_PROGRESS=3, /* Parameter value received but not yet set/accepted. A subsequent PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned immediately for parameters that take longer to set, indicating that the parameter was received and does not need to be resent. | */
};

//! PARAM_ACK ENUM_END
constexpr auto PARAM_ACK_ENUM_END = 4;

/** @brief Camera Modes. */
enum class CAMERA_MODE : uint8_t
{
    IMAGE=0, /* Camera is in image/photo capture mode. | */
    VIDEO=1, /* Camera is in video capture mode. | */
    IMAGE_SURVEY=2, /* Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys. | */
};

//! CAMERA_MODE ENUM_END
constexpr auto CAMERA_MODE_ENUM_END = 3;

/** @brief Reasons for denying an authorization request made with MAV_CMD_ARM_AUTHORIZATION_REQUEST. If the COMMAND_ACK result is MAV_RESULT_DENIED, this is used to set the reason in the result_param2 field. */
enum class MAV_ARM_AUTH_DENIED_REASON
{
    GENERIC=0, /* Not a specific reason | */
    NONE=1, /* Authorizer will send the error as string to GCS | */
    INVALID_WAYPOINT=2, /* At least one waypoint have a invalid value | */
    TIMEOUT=3, /* Timeout in the authorizer process(in case it depends on network) | */
    AIRSPACE_IN_USE=4, /* Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied. | */
    BAD_WEATHER=5, /* Weather is not good to fly | */
};

//! MAV_ARM_AUTH_DENIED_REASON ENUM_END
constexpr auto MAV_ARM_AUTH_DENIED_REASON_ENUM_END = 6;

/** @brief RC type. Used in MAV_CMD_START_RX_PAIR. */
enum class RC_TYPE
{
    SPEKTRUM=0, /* Spektrum | */
    CRSF=1, /* CRSF | */
};

//! RC_TYPE ENUM_END
constexpr auto RC_TYPE_ENUM_END = 2;

/** @brief RC sub-type of types defined in RC_TYPE. Used in MAV_CMD_START_RX_PAIR. Ignored if value does not correspond to the set RC_TYPE. */
enum class RC_SUB_TYPE
{
    SPEKTRUM_DSM2=0, /* Spektrum DSM2 | */
    SPEKTRUM_DSMX=1, /* Spektrum DSMX | */
    SPEKTRUM_DSMX8=2, /* Spektrum DSMX8 | */
};

//! RC_SUB_TYPE ENUM_END
constexpr auto RC_SUB_TYPE_ENUM_END = 3;

/** @brief RADIO_RC_CHANNELS flags (bitmask). */
enum class RADIO_RC_CHANNELS_FLAGS : uint16_t
{
    FAILSAFE=1, /* Failsafe is active. The content of the RC channels data in the RADIO_RC_CHANNELS message is implementation dependent. | */
    OUTDATED=2, /* Channel data may be out of date. This is set when the receiver is unable to validate incoming data from the transmitter and has therefore resent the last valid data it received. | */
};

//! RADIO_RC_CHANNELS_FLAGS ENUM_END
constexpr auto RADIO_RC_CHANNELS_FLAGS_ENUM_END = 3;

/** @brief Engine control options */
enum class ENGINE_CONTROL_OPTIONS
{
    ALLOW_START_WHILE_DISARMED=1, /* Allow starting the engine while disarmed (without changing the vehicle's armed state). This effectively arms just the ICE, without arming the vehicle to start other motors or propellers. | */
};

//! ENGINE_CONTROL_OPTIONS ENUM_END
constexpr auto ENGINE_CONTROL_OPTIONS_ENUM_END = 2;

/** @brief Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration. */
enum class POSITION_TARGET_TYPEMASK : uint16_t
{
    X_IGNORE=1, /* Ignore position x | */
    Y_IGNORE=2, /* Ignore position y | */
    Z_IGNORE=4, /* Ignore position z | */
    VX_IGNORE=8, /* Ignore velocity x | */
    VY_IGNORE=16, /* Ignore velocity y | */
    VZ_IGNORE=32, /* Ignore velocity z | */
    AX_IGNORE=64, /* Ignore acceleration x | */
    AY_IGNORE=128, /* Ignore acceleration y | */
    AZ_IGNORE=256, /* Ignore acceleration z | */
    FORCE_SET=512, /* Use force instead of acceleration | */
    YAW_IGNORE=1024, /* Ignore yaw | */
    YAW_RATE_IGNORE=2048, /* Ignore yaw rate | */
};

//! POSITION_TARGET_TYPEMASK ENUM_END
constexpr auto POSITION_TARGET_TYPEMASK_ENUM_END = 2049;

/** @brief Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored. */
enum class ATTITUDE_TARGET_TYPEMASK : uint8_t
{
    BODY_ROLL_RATE_IGNORE=1, /* Ignore body roll rate | */
    BODY_PITCH_RATE_IGNORE=2, /* Ignore body pitch rate | */
    BODY_YAW_RATE_IGNORE=4, /* Ignore body yaw rate | */
    THRUST_BODY_SET=32, /* Use 3D body thrust setpoint instead of throttle | */
    THROTTLE_IGNORE=64, /* Ignore throttle | */
    ATTITUDE_IGNORE=128, /* Ignore attitude | */
};

//! ATTITUDE_TARGET_TYPEMASK ENUM_END
constexpr auto ATTITUDE_TARGET_TYPEMASK_ENUM_END = 129;

/** @brief Airborne status of UAS. */
enum class UTM_FLIGHT_STATE : uint8_t
{
    UNKNOWN=1, /* The flight state can't be determined. | */
    GROUND=2, /* UAS on ground. | */
    AIRBORNE=3, /* UAS airborne. | */
    EMERGENCY=16, /* UAS is in an emergency flight state. | */
    NOCTRL=32, /* UAS has no active controls. | */
};

//! UTM_FLIGHT_STATE ENUM_END
constexpr auto UTM_FLIGHT_STATE_ENUM_END = 33;

/** @brief Flags for the global position report. */
enum class UTM_DATA_AVAIL_FLAGS : uint8_t
{
    TIME_VALID=1, /* The field time contains valid data. | */
    UAS_ID_AVAILABLE=2, /* The field uas_id contains valid data. | */
    POSITION_AVAILABLE=4, /* The fields lat, lon and h_acc contain valid data. | */
    ALTITUDE_AVAILABLE=8, /* The fields alt and v_acc contain valid data. | */
    RELATIVE_ALTITUDE_AVAILABLE=16, /* The field relative_alt contains valid data. | */
    HORIZONTAL_VELO_AVAILABLE=32, /* The fields vx and vy contain valid data. | */
    VERTICAL_VELO_AVAILABLE=64, /* The field vz contains valid data. | */
    NEXT_WAYPOINT_AVAILABLE=128, /* The fields next_lat, next_lon and next_alt contain valid data. | */
};

//! UTM_DATA_AVAIL_FLAGS ENUM_END
constexpr auto UTM_DATA_AVAIL_FLAGS_ENUM_END = 129;

/** @brief These flags encode the cellular network status */
enum class CELLULAR_STATUS_FLAG : uint8_t
{
    UNKNOWN=0, /* State unknown or not reportable. | */
    FAILED=1, /* Modem is unusable | */
    INITIALIZING=2, /* Modem is being initialized | */
    LOCKED=3, /* Modem is locked | */
    DISABLED=4, /* Modem is not enabled and is powered down | */
    DISABLING=5, /* Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state | */
    ENABLING=6, /* Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state | */
    ENABLED=7, /* Modem is enabled and powered on but not registered with a network provider and not available for data connections | */
    SEARCHING=8, /* Modem is searching for a network provider to register | */
    REGISTERED=9, /* Modem is registered with a network provider, and data connections and messaging may be available for use | */
    DISCONNECTING=10, /* Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated | */
    CONNECTING=11, /* Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered | */
    CONNECTED=12, /* One or more packet data bearers is active and connected | */
};

//! CELLULAR_STATUS_FLAG ENUM_END
constexpr auto CELLULAR_STATUS_FLAG_ENUM_END = 13;

/** @brief These flags are used to diagnose the failure state of CELLULAR_STATUS */
enum class CELLULAR_NETWORK_FAILED_REASON : uint8_t
{
    NONE=0, /* No error | */
    UNKNOWN=1, /* Error state is unknown | */
    SIM_MISSING=2, /* SIM is required for the modem but missing | */
    SIM_ERROR=3, /* SIM is available, but not usable for connection | */
};

//! CELLULAR_NETWORK_FAILED_REASON ENUM_END
constexpr auto CELLULAR_NETWORK_FAILED_REASON_ENUM_END = 4;

/** @brief Cellular network radio type */
enum class CELLULAR_NETWORK_RADIO_TYPE : uint8_t
{
    NONE=0, /*  | */
    GSM=1, /*  | */
    CDMA=2, /*  | */
    WCDMA=3, /*  | */
    LTE=4, /*  | */
};

//! CELLULAR_NETWORK_RADIO_TYPE ENUM_END
constexpr auto CELLULAR_NETWORK_RADIO_TYPE_ENUM_END = 5;

/** @brief Precision land modes (used in MAV_CMD_NAV_LAND). */
enum class PRECISION_LAND_MODE
{
    DISABLED=0, /* Normal (non-precision) landing. | */
    OPPORTUNISTIC=1, /* Use precision landing if beacon detected when land command accepted, otherwise land normally. | */
    REQUIRED=2, /* Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found). | */
};

//! PRECISION_LAND_MODE ENUM_END
constexpr auto PRECISION_LAND_MODE_ENUM_END = 3;

/** @brief Parachute actions. Trigger release and enable/disable auto-release. */
enum class PARACHUTE_ACTION
{
    DISABLE=0, /* Disable auto-release of parachute (i.e. release triggered by crash detectors). | */
    ENABLE=1, /* Enable auto-release of parachute. | */
    RELEASE=2, /* Release parachute and kill motors. | */
};

//! PARACHUTE_ACTION ENUM_END
constexpr auto PARACHUTE_ACTION_ENUM_END = 3;

/** @brief  */
enum class MAV_TUNNEL_PAYLOAD_TYPE : uint16_t
{
    UNKNOWN=0, /* Encoding of payload unknown. | */
    STORM32_RESERVED0=200, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED1=201, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED2=202, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED3=203, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED4=204, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED5=205, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED6=206, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED7=207, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED8=208, /* Registered for STorM32 gimbal controller. | */
    STORM32_RESERVED9=209, /* Registered for STorM32 gimbal controller. | */
    MODALAI_REMOTE_OSD=210, /* Registered for ModalAI remote OSD protocol. | */
    MODALAI_ESC_UART_PASSTHRU=211, /* Registered for ModalAI ESC UART passthru protocol. | */
    MODALAI_IO_UART_PASSTHRU=212, /* Registered for ModalAI vendor use. | */
};

//! MAV_TUNNEL_PAYLOAD_TYPE ENUM_END
constexpr auto MAV_TUNNEL_PAYLOAD_TYPE_ENUM_END = 213;

/** @brief  */
enum class MAV_ODID_ID_TYPE : uint8_t
{
    NONE=0, /* No type defined. | */
    SERIAL_NUMBER=1, /* Manufacturer Serial Number (ANSI/CTA-2063 format). | */
    CAA_REGISTRATION_ID=2, /* CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID]. | */
    UTM_ASSIGNED_UUID=3, /* UTM (Unmanned Traffic Management) assigned UUID (RFC4122). | */
    SPECIFIC_SESSION_ID=4, /* A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of uas_id and these type values are managed by ICAO. | */
};

//! MAV_ODID_ID_TYPE ENUM_END
constexpr auto MAV_ODID_ID_TYPE_ENUM_END = 5;

/** @brief  */
enum class MAV_ODID_UA_TYPE : uint8_t
{
    NONE=0, /* No UA (Unmanned Aircraft) type defined. | */
    AEROPLANE=1, /* Aeroplane/Airplane. Fixed wing. | */
    HELICOPTER_OR_MULTIROTOR=2, /* Helicopter or multirotor. | */
    GYROPLANE=3, /* Gyroplane. | */
    HYBRID_LIFT=4, /* VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically. | */
    ORNITHOPTER=5, /* Ornithopter. | */
    GLIDER=6, /* Glider. | */
    KITE=7, /* Kite. | */
    FREE_BALLOON=8, /* Free Balloon. | */
    CAPTIVE_BALLOON=9, /* Captive Balloon. | */
    AIRSHIP=10, /* Airship. E.g. a blimp. | */
    FREE_FALL_PARACHUTE=11, /* Free Fall/Parachute (unpowered). | */
    ROCKET=12, /* Rocket. | */
    TETHERED_POWERED_AIRCRAFT=13, /* Tethered powered aircraft. | */
    GROUND_OBSTACLE=14, /* Ground Obstacle. | */
    OTHER=15, /* Other type of aircraft not listed earlier. | */
};

//! MAV_ODID_UA_TYPE ENUM_END
constexpr auto MAV_ODID_UA_TYPE_ENUM_END = 16;

/** @brief  */
enum class MAV_ODID_STATUS : uint8_t
{
    UNDECLARED=0, /* The status of the (UA) Unmanned Aircraft is undefined. | */
    GROUND=1, /* The UA is on the ground. | */
    AIRBORNE=2, /* The UA is in the air. | */
    EMERGENCY=3, /* The UA is having an emergency. | */
    REMOTE_ID_SYSTEM_FAILURE=4, /* The remote ID system is failing or unreliable in some way. | */
};

//! MAV_ODID_STATUS ENUM_END
constexpr auto MAV_ODID_STATUS_ENUM_END = 5;

/** @brief  */
enum class MAV_ODID_HEIGHT_REF : uint8_t
{
    OVER_TAKEOFF=0, /* The height field is relative to the take-off location. | */
    OVER_GROUND=1, /* The height field is relative to ground. | */
};

//! MAV_ODID_HEIGHT_REF ENUM_END
constexpr auto MAV_ODID_HEIGHT_REF_ENUM_END = 2;

/** @brief  */
enum class MAV_ODID_HOR_ACC : uint8_t
{
    UNKNOWN=0, /* The horizontal accuracy is unknown. | */
    ACC_10NM=1, /* The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km. | */
    ACC_4NM=2, /* The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km. | */
    ACC_2NM=3, /* The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km. | */
    ACC_1NM=4, /* The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km. | */
    ACC_0_5NM=5, /* The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m. | */
    ACC_0_3NM=6, /* The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m. | */
    ACC_0_1NM=7, /* The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m. | */
    ACC_0_05NM=8, /* The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m. | */
    ACC_30_METER=9, /* The horizontal accuracy is smaller than 30 meter. | */
    ACC_10_METER=10, /* The horizontal accuracy is smaller than 10 meter. | */
    ACC_3_METER=11, /* The horizontal accuracy is smaller than 3 meter. | */
    ACC_1_METER=12, /* The horizontal accuracy is smaller than 1 meter. | */
};

//! MAV_ODID_HOR_ACC ENUM_END
constexpr auto MAV_ODID_HOR_ACC_ENUM_END = 13;

/** @brief  */
enum class MAV_ODID_VER_ACC : uint8_t
{
    UNKNOWN=0, /* The vertical accuracy is unknown. | */
    ACC_150_METER=1, /* The vertical accuracy is smaller than 150 meter. | */
    ACC_45_METER=2, /* The vertical accuracy is smaller than 45 meter. | */
    ACC_25_METER=3, /* The vertical accuracy is smaller than 25 meter. | */
    ACC_10_METER=4, /* The vertical accuracy is smaller than 10 meter. | */
    ACC_3_METER=5, /* The vertical accuracy is smaller than 3 meter. | */
    ACC_1_METER=6, /* The vertical accuracy is smaller than 1 meter. | */
};

//! MAV_ODID_VER_ACC ENUM_END
constexpr auto MAV_ODID_VER_ACC_ENUM_END = 7;

/** @brief  */
enum class MAV_ODID_SPEED_ACC : uint8_t
{
    UNKNOWN=0, /* The speed accuracy is unknown. | */
    ACC_10_METERS_PER_SECOND=1, /* The speed accuracy is smaller than 10 meters per second. | */
    ACC_3_METERS_PER_SECOND=2, /* The speed accuracy is smaller than 3 meters per second. | */
    ACC_1_METERS_PER_SECOND=3, /* The speed accuracy is smaller than 1 meters per second. | */
    ACC_0_3_METERS_PER_SECOND=4, /* The speed accuracy is smaller than 0.3 meters per second. | */
};

//! MAV_ODID_SPEED_ACC ENUM_END
constexpr auto MAV_ODID_SPEED_ACC_ENUM_END = 5;

/** @brief  */
enum class MAV_ODID_TIME_ACC : uint8_t
{
    UNKNOWN=0, /* The timestamp accuracy is unknown. | */
    ACC_0_1_SECOND=1, /* The timestamp accuracy is smaller than or equal to 0.1 second. | */
    ACC_0_2_SECOND=2, /* The timestamp accuracy is smaller than or equal to 0.2 second. | */
    ACC_0_3_SECOND=3, /* The timestamp accuracy is smaller than or equal to 0.3 second. | */
    ACC_0_4_SECOND=4, /* The timestamp accuracy is smaller than or equal to 0.4 second. | */
    ACC_0_5_SECOND=5, /* The timestamp accuracy is smaller than or equal to 0.5 second. | */
    ACC_0_6_SECOND=6, /* The timestamp accuracy is smaller than or equal to 0.6 second. | */
    ACC_0_7_SECOND=7, /* The timestamp accuracy is smaller than or equal to 0.7 second. | */
    ACC_0_8_SECOND=8, /* The timestamp accuracy is smaller than or equal to 0.8 second. | */
    ACC_0_9_SECOND=9, /* The timestamp accuracy is smaller than or equal to 0.9 second. | */
    ACC_1_0_SECOND=10, /* The timestamp accuracy is smaller than or equal to 1.0 second. | */
    ACC_1_1_SECOND=11, /* The timestamp accuracy is smaller than or equal to 1.1 second. | */
    ACC_1_2_SECOND=12, /* The timestamp accuracy is smaller than or equal to 1.2 second. | */
    ACC_1_3_SECOND=13, /* The timestamp accuracy is smaller than or equal to 1.3 second. | */
    ACC_1_4_SECOND=14, /* The timestamp accuracy is smaller than or equal to 1.4 second. | */
    ACC_1_5_SECOND=15, /* The timestamp accuracy is smaller than or equal to 1.5 second. | */
};

//! MAV_ODID_TIME_ACC ENUM_END
constexpr auto MAV_ODID_TIME_ACC_ENUM_END = 16;

/** @brief  */
enum class MAV_ODID_AUTH_TYPE : uint8_t
{
    NONE=0, /* No authentication type is specified. | */
    UAS_ID_SIGNATURE=1, /* Signature for the UAS (Unmanned Aircraft System) ID. | */
    OPERATOR_ID_SIGNATURE=2, /* Signature for the Operator ID. | */
    MESSAGE_SET_SIGNATURE=3, /* Signature for the entire message set. | */
    NETWORK_REMOTE_ID=4, /* Authentication is provided by Network Remote ID. | */
    SPECIFIC_AUTHENTICATION=5, /* The exact authentication type is indicated by the first byte of authentication_data and these type values are managed by ICAO. | */
};

//! MAV_ODID_AUTH_TYPE ENUM_END
constexpr auto MAV_ODID_AUTH_TYPE_ENUM_END = 6;

/** @brief  */
enum class MAV_ODID_DESC_TYPE : uint8_t
{
    TEXT=0, /* Optional free-form text description of the purpose of the flight. | */
    EMERGENCY=1, /* Optional additional clarification when status == MAV_ODID_STATUS_EMERGENCY. | */
    EXTENDED_STATUS=2, /* Optional additional clarification when status != MAV_ODID_STATUS_EMERGENCY. | */
};

//! MAV_ODID_DESC_TYPE ENUM_END
constexpr auto MAV_ODID_DESC_TYPE_ENUM_END = 3;

/** @brief  */
enum class MAV_ODID_OPERATOR_LOCATION_TYPE : uint8_t
{
    TAKEOFF=0, /* The location/altitude of the operator is the same as the take-off location. | */
    LIVE_GNSS=1, /* The location/altitude of the operator is dynamic. E.g. based on live GNSS data. | */
    FIXED=2, /* The location/altitude of the operator are fixed values. | */
};

//! MAV_ODID_OPERATOR_LOCATION_TYPE ENUM_END
constexpr auto MAV_ODID_OPERATOR_LOCATION_TYPE_ENUM_END = 3;

/** @brief  */
enum class MAV_ODID_CLASSIFICATION_TYPE : uint8_t
{
    UNDECLARED=0, /* The classification type for the UA is undeclared. | */
    EU=1, /* The classification type for the UA follows EU (European Union) specifications. | */
};

//! MAV_ODID_CLASSIFICATION_TYPE ENUM_END
constexpr auto MAV_ODID_CLASSIFICATION_TYPE_ENUM_END = 2;

/** @brief  */
enum class MAV_ODID_CATEGORY_EU : uint8_t
{
    UNDECLARED=0, /* The category for the UA, according to the EU specification, is undeclared. | */
    OPEN=1, /* The category for the UA, according to the EU specification, is the Open category. | */
    SPECIFIC=2, /* The category for the UA, according to the EU specification, is the Specific category. | */
    CERTIFIED=3, /* The category for the UA, according to the EU specification, is the Certified category. | */
};

//! MAV_ODID_CATEGORY_EU ENUM_END
constexpr auto MAV_ODID_CATEGORY_EU_ENUM_END = 4;

/** @brief  */
enum class MAV_ODID_CLASS_EU : uint8_t
{
    UNDECLARED=0, /* The class for the UA, according to the EU specification, is undeclared. | */
    CLASS_0=1, /* The class for the UA, according to the EU specification, is Class 0. | */
    CLASS_1=2, /* The class for the UA, according to the EU specification, is Class 1. | */
    CLASS_2=3, /* The class for the UA, according to the EU specification, is Class 2. | */
    CLASS_3=4, /* The class for the UA, according to the EU specification, is Class 3. | */
    CLASS_4=5, /* The class for the UA, according to the EU specification, is Class 4. | */
    CLASS_5=6, /* The class for the UA, according to the EU specification, is Class 5. | */
    CLASS_6=7, /* The class for the UA, according to the EU specification, is Class 6. | */
};

//! MAV_ODID_CLASS_EU ENUM_END
constexpr auto MAV_ODID_CLASS_EU_ENUM_END = 8;

/** @brief  */
enum class MAV_ODID_OPERATOR_ID_TYPE : uint8_t
{
    CAA=0, /* CAA (Civil Aviation Authority) registered operator ID. | */
};

//! MAV_ODID_OPERATOR_ID_TYPE ENUM_END
constexpr auto MAV_ODID_OPERATOR_ID_TYPE_ENUM_END = 1;

/** @brief  */
enum class MAV_ODID_ARM_STATUS : uint8_t
{
    GOOD_TO_ARM=0, /* Passing arming checks. | */
    PRE_ARM_FAIL_GENERIC=1, /* Generic arming failure, see error string for details. | */
};

//! MAV_ODID_ARM_STATUS ENUM_END
constexpr auto MAV_ODID_ARM_STATUS_ENUM_END = 2;

/** @brief Tune formats (used for vehicle buzzer/tone generation). */
enum class TUNE_FORMAT : uint32_t
{
    QBASIC1_1=1, /* Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm. | */
    MML_MODERN=2, /* Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML. | */
};

//! TUNE_FORMAT ENUM_END
constexpr auto TUNE_FORMAT_ENUM_END = 3;

/** @brief Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html */
enum class AIS_TYPE : uint8_t
{
    UNKNOWN=0, /* Not available (default). | */
    RESERVED_1=1, /*  | */
    RESERVED_2=2, /*  | */
    RESERVED_3=3, /*  | */
    RESERVED_4=4, /*  | */
    RESERVED_5=5, /*  | */
    RESERVED_6=6, /*  | */
    RESERVED_7=7, /*  | */
    RESERVED_8=8, /*  | */
    RESERVED_9=9, /*  | */
    RESERVED_10=10, /*  | */
    RESERVED_11=11, /*  | */
    RESERVED_12=12, /*  | */
    RESERVED_13=13, /*  | */
    RESERVED_14=14, /*  | */
    RESERVED_15=15, /*  | */
    RESERVED_16=16, /*  | */
    RESERVED_17=17, /*  | */
    RESERVED_18=18, /*  | */
    RESERVED_19=19, /*  | */
    WIG=20, /* Wing In Ground effect. | */
    WIG_HAZARDOUS_A=21, /*  | */
    WIG_HAZARDOUS_B=22, /*  | */
    WIG_HAZARDOUS_C=23, /*  | */
    WIG_HAZARDOUS_D=24, /*  | */
    WIG_RESERVED_1=25, /*  | */
    WIG_RESERVED_2=26, /*  | */
    WIG_RESERVED_3=27, /*  | */
    WIG_RESERVED_4=28, /*  | */
    WIG_RESERVED_5=29, /*  | */
    FISHING=30, /*  | */
    TOWING=31, /*  | */
    TOWING_LARGE=32, /* Towing: length exceeds 200m or breadth exceeds 25m. | */
    DREDGING=33, /* Dredging or other underwater ops. | */
    DIVING=34, /*  | */
    MILITARY=35, /*  | */
    SAILING=36, /*  | */
    PLEASURE=37, /*  | */
    RESERVED_20=38, /*  | */
    RESERVED_21=39, /*  | */
    HSC=40, /* High Speed Craft. | */
    HSC_HAZARDOUS_A=41, /*  | */
    HSC_HAZARDOUS_B=42, /*  | */
    HSC_HAZARDOUS_C=43, /*  | */
    HSC_HAZARDOUS_D=44, /*  | */
    HSC_RESERVED_1=45, /*  | */
    HSC_RESERVED_2=46, /*  | */
    HSC_RESERVED_3=47, /*  | */
    HSC_RESERVED_4=48, /*  | */
    HSC_UNKNOWN=49, /*  | */
    PILOT=50, /*  | */
    SAR=51, /* Search And Rescue vessel. | */
    TUG=52, /*  | */
    PORT_TENDER=53, /*  | */
    ANTI_POLLUTION=54, /* Anti-pollution equipment. | */
    LAW_ENFORCEMENT=55, /*  | */
    SPARE_LOCAL_1=56, /*  | */
    SPARE_LOCAL_2=57, /*  | */
    MEDICAL_TRANSPORT=58, /*  | */
    NONECOMBATANT=59, /* Noncombatant ship according to RR Resolution No. 18. | */
    PASSENGER=60, /*  | */
    PASSENGER_HAZARDOUS_A=61, /*  | */
    PASSENGER_HAZARDOUS_B=62, /*  | */
    PASSENGER_HAZARDOUS_C=63, /*  | */
    PASSENGER_HAZARDOUS_D=64, /*  | */
    PASSENGER_RESERVED_1=65, /*  | */
    PASSENGER_RESERVED_2=66, /*  | */
    PASSENGER_RESERVED_3=67, /*  | */
    PASSENGER_RESERVED_4=68, /*  | */
    PASSENGER_UNKNOWN=69, /*  | */
    CARGO=70, /*  | */
    CARGO_HAZARDOUS_A=71, /*  | */
    CARGO_HAZARDOUS_B=72, /*  | */
    CARGO_HAZARDOUS_C=73, /*  | */
    CARGO_HAZARDOUS_D=74, /*  | */
    CARGO_RESERVED_1=75, /*  | */
    CARGO_RESERVED_2=76, /*  | */
    CARGO_RESERVED_3=77, /*  | */
    CARGO_RESERVED_4=78, /*  | */
    CARGO_UNKNOWN=79, /*  | */
    TANKER=80, /*  | */
    TANKER_HAZARDOUS_A=81, /*  | */
    TANKER_HAZARDOUS_B=82, /*  | */
    TANKER_HAZARDOUS_C=83, /*  | */
    TANKER_HAZARDOUS_D=84, /*  | */
    TANKER_RESERVED_1=85, /*  | */
    TANKER_RESERVED_2=86, /*  | */
    TANKER_RESERVED_3=87, /*  | */
    TANKER_RESERVED_4=88, /*  | */
    TANKER_UNKNOWN=89, /*  | */
    OTHER=90, /*  | */
    OTHER_HAZARDOUS_A=91, /*  | */
    OTHER_HAZARDOUS_B=92, /*  | */
    OTHER_HAZARDOUS_C=93, /*  | */
    OTHER_HAZARDOUS_D=94, /*  | */
    OTHER_RESERVED_1=95, /*  | */
    OTHER_RESERVED_2=96, /*  | */
    OTHER_RESERVED_3=97, /*  | */
    OTHER_RESERVED_4=98, /*  | */
    OTHER_UNKNOWN=99, /*  | */
};

//! AIS_TYPE ENUM_END
constexpr auto AIS_TYPE_ENUM_END = 100;

/** @brief Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html */
enum class AIS_NAV_STATUS : uint8_t
{
    UNDER_WAY=0, /* Under way using engine. | */
    ANCHORED=1, /*  | */
    UN_COMMANDED=2, /*  | */
    RESTRICTED_MANOEUVERABILITY=3, /*  | */
    DRAUGHT_CONSTRAINED=4, /*  | */
    MOORED=5, /*  | */
    AGROUND=6, /*  | */
    FISHING=7, /*  | */
    SAILING=8, /*  | */
    RESERVED_HSC=9, /*  | */
    RESERVED_WIG=10, /*  | */
    RESERVED_1=11, /*  | */
    RESERVED_2=12, /*  | */
    RESERVED_3=13, /*  | */
    AIS_SART=14, /* Search And Rescue Transponder. | */
    UNKNOWN=15, /* Not available (default). | */
};

//! AIS_NAV_STATUS ENUM_END
constexpr auto AIS_NAV_STATUS_ENUM_END = 16;

/** @brief These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message fields. When set, the data is valid. */
enum class AIS_FLAGS : uint16_t
{
    POSITION_ACCURACY=1, /* 1 = High (Position accuracy less than or equal to 10m), 0 = Low (position accuracy greater than 10m). | */
    VALID_COG=2, /* The COG field contains valid data | */
    VALID_VELOCITY=4, /* The velocity field contains valid data | */
    HIGH_VELOCITY=8, /* 1 = Velocity over 52.5765m/s (102.2 knots) | */
    VALID_TURN_RATE=16, /* The turn_rate field contains valid data | */
    TURN_RATE_SIGN_ONLY=32, /* Only the sign of the returned turn_rate value is valid. The actual turn rate is either greater than 5deg/30s or less than -5deg/30s. | */
    VALID_DIMENSIONS=64, /*  | */
    LARGE_BOW_DIMENSION=128, /* Distance to bow is greater than or equal to 511m | */
    LARGE_STERN_DIMENSION=256, /* Distance to stern is greater than or equal to 511m | */
    LARGE_PORT_DIMENSION=512, /* Distance to port side is greater than or equal to 63m | */
    LARGE_STARBOARD_DIMENSION=1024, /* Distance to starboard side is greater than or equal to 63m | */
    VALID_CALLSIGN=2048, /* The callsign field contains valid data | */
    VALID_NAME=4096, /* The name field contains valid data | */
};

//! AIS_FLAGS ENUM_END
constexpr auto AIS_FLAGS_ENUM_END = 4097;

/** @brief List of possible units where failures can be injected. */
enum class FAILURE_UNIT
{
    SENSOR_GYRO=0, /*  | */
    SENSOR_ACCEL=1, /*  | */
    SENSOR_MAG=2, /*  | */
    SENSOR_BARO=3, /*  | */
    SENSOR_GPS=4, /*  | */
    SENSOR_OPTICAL_FLOW=5, /*  | */
    SENSOR_VIO=6, /*  | */
    SENSOR_DISTANCE_SENSOR=7, /*  | */
    SENSOR_AIRSPEED=8, /*  | */
    SYSTEM_BATTERY=100, /*  | */
    SYSTEM_MOTOR=101, /* Interrupts the commanded output to the motor. | */
    SYSTEM_SERVO=102, /*  | */
    SYSTEM_AVOIDANCE=103, /*  | */
    SYSTEM_RC_SIGNAL=104, /*  | */
    SYSTEM_MAVLINK_SIGNAL=105, /*  | */
    SYSTEM_ESC=106, /* Interrupts the telemetry reported by the ESC. | */
    SYSTEM_TRAFFIC_AVOIDANCE=107, /* Traffic avoidance system like ADS-B or FLARM. | */
    DATALINK_LTE=150, /* Data link over a cellular (LTE) connection. | */
    DATALINK_WIFI=151, /* Data link over a Wi-Fi connection. | */
    DATALINK_TELEM_RADIO=152, /* Data link over a telemetry radio, e.g. a SiK radio. | */
    BUS_CAN=200, /* CAN bus. Instance is the bus number, e.g. 1 for CAN1. | */
    BUS_I2C=201, /* I2C bus. Instance is the bus number, e.g. 1 for I2C1. | */
};

//! FAILURE_UNIT ENUM_END
constexpr auto FAILURE_UNIT_ENUM_END = 202;

/** @brief List of possible failure type to inject. */
enum class FAILURE_TYPE
{
    OK=0, /* No failure injected, used to reset a previous failure. | */
    OFF=1, /* Sets unit off, so completely non-responsive. | */
    STUCK=2, /* Unit is stuck e.g. keeps reporting the same value. | */
    GARBAGE=3, /* Unit is reporting complete garbage. | */
    WRONG=4, /* Unit is consistently wrong. | */
    SLOW=5, /* Unit is slow, so e.g. reporting at slower than expected rate. | */
    DELAYED=6, /* Data of unit is delayed in time. | */
    INTERMITTENT=7, /* Unit is sometimes working, sometimes not. | */
    DRIFT=8, /* Unit is publishing plausible values but drifting away from true values. | */
};

//! FAILURE_TYPE ENUM_END
constexpr auto FAILURE_TYPE_ENUM_END = 9;

/** @brief  */
enum class NAV_VTOL_LAND_OPTIONS
{
    DEFAULT=0, /* Default autopilot landing behaviour. | */
    FW_SPIRAL_APPROACH=1, /* Use a fixed wing spiral desent approach before landing. | */
    FW_APPROACH=2, /* Use a fixed wing approach before detransitioning and landing vertically. | */
};

//! NAV_VTOL_LAND_OPTIONS ENUM_END
constexpr auto NAV_VTOL_LAND_OPTIONS_ENUM_END = 3;

/** @brief Winch status flags used in WINCH_STATUS */
enum class MAV_WINCH_STATUS_FLAG : uint32_t
{
    HEALTHY=1, /* Winch is healthy | */
    FULLY_RETRACTED=2, /* Winch line is fully retracted | */
    MOVING=4, /* Winch motor is moving | */
    CLUTCH_DISENGAGED=8, /* Winch clutch is disengaged. The motor is moving freely and not driving the winch. | */
    LOCKED=16, /* Winch is locked by locking mechanism. | */
    DROPPING=32, /* Winch is gravity dropping payload. | */
    ARRESTING=64, /* Winch is arresting payload descent. | */
    GROUND_SENSE=128, /* Winch is using torque measurements to sense the ground. | */
    RETRACTING=256, /* Winch is returning to the fully retracted position. | */
    REDELIVER=512, /* Winch is redelivering the payload. This is a failover state if the line tension goes above a threshold during RETRACTING. | */
    ABANDON_LINE=1024, /* Winch is abandoning the line and possibly payload. Winch unspools the entire calculated line length. This is a failover state from REDELIVER if the number of attempts exceeds a threshold. | */
    LOCKING=2048, /* Winch is engaging the locking mechanism. | */
    LOAD_LINE=4096, /* Winch is spooling on line. | */
    LOAD_PAYLOAD=8192, /* Winch is loading a payload. | */
};

//! MAV_WINCH_STATUS_FLAG ENUM_END
constexpr auto MAV_WINCH_STATUS_FLAG_ENUM_END = 8193;

/** @brief  */
enum class MAG_CAL_STATUS : uint8_t
{
    NOT_STARTED=0, /*  | */
    WAITING_TO_START=1, /*  | */
    RUNNING_STEP_ONE=2, /*  | */
    RUNNING_STEP_TWO=3, /*  | */
    SUCCESS=4, /*  | */
    FAILED=5, /*  | */
    FAILED_ORIENTATION=6, /* Compass calibration failed: the vehicle orientation is outside the required tolerance. | */
    FAILED_RADIUS=7, /* Compass calibration failed: the radius of the fitted sphere is unrealistically small or large. | */
    FAILED_OFFSETS=8, /* Compass calibration failed: offset magnitude too large. | */
    FAILED_DIAG_SCALING=9, /* Compass calibration failed: diagonal or off-diagonal scaling values out of valid range. | */
    FAILED_RESIDUALS_HIGH=10, /* Compass calibration failed: fitness (RMS residual) exceeds tolerance. | */
};

//! MAG_CAL_STATUS ENUM_END
constexpr auto MAG_CAL_STATUS_ENUM_END = 11;

/** @brief Reason for an event error response. */
enum class MAV_EVENT_ERROR_REASON : uint8_t
{
    UNAVAILABLE=0, /* The requested event is not available (anymore). | */
};

//! MAV_EVENT_ERROR_REASON ENUM_END
constexpr auto MAV_EVENT_ERROR_REASON_ENUM_END = 1;

/** @brief Flags for CURRENT_EVENT_SEQUENCE. */
enum class MAV_EVENT_CURRENT_SEQUENCE_FLAGS : uint8_t
{
    RESET=1, /* A sequence reset has happened (e.g. vehicle reboot). | */
};

//! MAV_EVENT_CURRENT_SEQUENCE_FLAGS ENUM_END
constexpr auto MAV_EVENT_CURRENT_SEQUENCE_FLAGS_ENUM_END = 2;

/** @brief Flags in the HIL_SENSOR message indicate which fields have updated since the last message */
enum class HIL_SENSOR_UPDATED_FLAGS : uint32_t
{
    XACC=1, /* The value in the xacc field has been updated | */
    YACC=2, /* The value in the yacc field has been updated | */
    ZACC=4, /* The value in the zacc field has been updated | */
    XGYRO=8, /* The value in the xgyro field has been updated | */
    YGYRO=16, /* The value in the ygyro field has been updated | */
    ZGYRO=32, /* The value in the zgyro field has been updated | */
    XMAG=64, /* The value in the xmag field has been updated | */
    YMAG=128, /* The value in the ymag field has been updated | */
    ZMAG=256, /* The value in the zmag field has been updated | */
    ABS_PRESSURE=512, /* The value in the abs_pressure field has been updated | */
    DIFF_PRESSURE=1024, /* The value in the diff_pressure field has been updated | */
    PRESSURE_ALT=2048, /* The value in the pressure_alt field has been updated | */
    TEMPERATURE=4096, /* The value in the temperature field has been updated | */
    RESET=2147483648, /* Full reset of attitude/position/velocities/etc was performed in sim (Bit 31). | */
};

//! HIL_SENSOR_UPDATED_FLAGS ENUM_END
constexpr auto HIL_SENSOR_UPDATED_FLAGS_ENUM_END = 2147483649;

/** @brief Flags in the HIGHRES_IMU message indicate which fields have updated since the last message */
enum class HIGHRES_IMU_UPDATED_FLAGS : uint16_t
{
    XACC=1, /* The value in the xacc field has been updated | */
    YACC=2, /* The value in the yacc field has been updated | */
    ZACC=4, /* The value in the zacc field has been updated since | */
    XGYRO=8, /* The value in the xgyro field has been updated | */
    YGYRO=16, /* The value in the ygyro field has been updated | */
    ZGYRO=32, /* The value in the zgyro field has been updated | */
    XMAG=64, /* The value in the xmag field has been updated | */
    YMAG=128, /* The value in the ymag field has been updated | */
    ZMAG=256, /* The value in the zmag field has been updated | */
    ABS_PRESSURE=512, /* The value in the abs_pressure field has been updated | */
    DIFF_PRESSURE=1024, /* The value in the diff_pressure field has been updated | */
    PRESSURE_ALT=2048, /* The value in the pressure_alt field has been updated | */
    TEMPERATURE=4096, /* The value in the temperature field has been updated | */
};

//! HIGHRES_IMU_UPDATED_FLAGS ENUM_END
constexpr auto HIGHRES_IMU_UPDATED_FLAGS_ENUM_END = 4097;

/** @brief  */
enum class CAN_FILTER_OP : uint8_t
{
    REPLACE=0, /*  | */
    ADD=1, /*  | */
    REMOVE=2, /*  | */
};

//! CAN_FILTER_OP ENUM_END
constexpr auto CAN_FILTER_OP_ENUM_END = 3;

/** @brief MAV FTP error codes (may be used in FILE_TRANSFER_PROTOCOL). See https://mavlink.io/en/services/ftp.html. */
enum class MAV_FTP_ERR
{
    NONE=0, /* None: No error | */
    FAIL=1, /* Fail: Unknown failure | */
    FAILERRNO=2, /* FailErrno: Command failed, Err number sent back in PayloadHeader.data[1].
		This is a file-system error number understood by the server operating system. | */
    INVALIDDATASIZE=3, /* InvalidDataSize: Payload size is invalid | */
    INVALIDSESSION=4, /* InvalidSession: Session is not currently open | */
    NOSESSIONSAVAILABLE=5, /* NoSessionsAvailable: All available sessions are already in use | */
    EOF_=6, /* EOF: Offset past end of file for ListDirectory and ReadFile commands | */
    UNKNOWNCOMMAND=7, /* UnknownCommand: Unknown command / opcode | */
    FILEEXISTS=8, /* FileExists: File/directory already exists | */
    FILEPROTECTED=9, /* FileProtected: File/directory is write protected | */
    FILENOTFOUND=10, /* FileNotFound: File/directory not found | */
};

//! MAV_FTP_ERR ENUM_END
constexpr auto MAV_FTP_ERR_ENUM_END = 11;

/** @brief MAV FTP opcodes (may be used in FILE_TRANSFER_PROTOCOL). See https://mavlink.io/en/services/ftp.html. */
enum class MAV_FTP_OPCODE
{
    NONE=0, /* None. Ignored, always ACKed | */
    TERMINATESESSION=1, /* TerminateSession: Terminates open Read session | */
    RESETSESSION=2, /* ResetSessions: Terminates all open read sessions | */
    LISTDIRECTORY=3, /* ListDirectory. List files and directories in path from offset | */
    OPENFILERO=4, /* OpenFileRO: Opens file at path for reading, returns session | */
    READFILE=5, /* ReadFile: Reads size bytes from offset in session | */
    CREATEFILE=6, /* CreateFile: Creates file at path for writing, returns session | */
    WRITEFILE=7, /* WriteFile: Writes size bytes to offset in session | */
    REMOVEFILE=8, /* RemoveFile: Remove file at path | */
    CREATEDIRECTORY=9, /* CreateDirectory: Creates directory at path | */
    REMOVEDIRECTORY=10, /* RemoveDirectory: Removes directory at path. The directory must be empty. | */
    OPENFILEWO=11, /* OpenFileWO: Opens file at path for writing, returns session | */
    TRUNCATEFILE=12, /* TruncateFile: Truncate file at path to offset length | */
    RENAME=13, /* Rename: Rename path1 to path2 | */
    CALCFILECRC=14, /* CalcFileCRC32: Calculate CRC32 for file at path | */
    BURSTREADFILE=15, /* BurstReadFile: Burst download session file | */
    LISTDIRECTORYWITHTIME=16, /* ListDirectoryWithTime: List files and directories, along with last-modification timestamps, in path from offset. This is the same as ListDirectory except for the addition of timestamps. Servers that do not support this opcode respond with a NAK (MAV_FTP_ERR_UNKNOWNCOMMAND). | */
    ACK=128, /* ACK: ACK response | */
    NAK=129, /* NAK: NAK response | */
};

//! MAV_FTP_OPCODE ENUM_END
constexpr auto MAV_FTP_OPCODE_ENUM_END = 130;

/** @brief 
        States of the mission state machine.
        Note that these states are independent of whether the mission is in a mode that can execute mission items or not (is suspended).
        They may not all be relevant on all vehicles.
       */
enum class MISSION_STATE : uint8_t
{
    UNKNOWN=0, /* The mission status reporting is not supported. | */
    NO_MISSION=1, /* No mission on the vehicle. | */
    NOT_STARTED=2, /* Mission has not started. This is the case after a mission has uploaded but not yet started executing. | */
    ACTIVE=3, /* Mission is active, and will execute mission items when in auto mode. | */
    PAUSED=4, /* Mission is paused when in auto mode. | */
    COMPLETE=5, /* Mission has executed all mission items. | */
};

//! MISSION_STATE ENUM_END
constexpr auto MISSION_STATE_ENUM_END = 6;

/** @brief 
	Possible safety switch states.
       */
enum class SAFETY_SWITCH_STATE
{
    SAFE=0, /* Safety switch is engaged and vehicle should be safe to approach. | */
    DANGEROUS=1, /* Safety switch is NOT engaged and motors, propellers and other actuators should be considered active. | */
};

//! SAFETY_SWITCH_STATE ENUM_END
constexpr auto SAFETY_SWITCH_STATE_ENUM_END = 2;

/** @brief Modes of illuminator */
enum class ILLUMINATOR_MODE : uint8_t
{
    UNKNOWN=0, /* Illuminator mode is not specified/unknown | */
    INTERNAL_CONTROL=1, /* Illuminator behavior is controlled by MAV_CMD_DO_ILLUMINATOR_CONFIGURE settings | */
    EXTERNAL_SYNC=2, /* Illuminator behavior is controlled by external factors: e.g. an external hardware signal | */
};

//! ILLUMINATOR_MODE ENUM_END
constexpr auto ILLUMINATOR_MODE_ENUM_END = 3;

/** @brief Illuminator module error flags (bitmap, 0 means no error) */
enum class ILLUMINATOR_ERROR_FLAGS : uint32_t
{
    THERMAL_THROTTLING=1, /* Illuminator thermal throttling error. | */
    OVER_TEMPERATURE_SHUTDOWN=2, /* Illuminator over temperature shutdown error. | */
    THERMISTOR_FAILURE=4, /* Illuminator thermistor failure. | */
};

//! ILLUMINATOR_ERROR_FLAGS ENUM_END
constexpr auto ILLUMINATOR_ERROR_FLAGS_ENUM_END = 5;

/** @brief Standard modes with a well understood meaning across flight stacks and vehicle types.
        For example, most flight stack have the concept of a "return" or "RTL" mode that takes a vehicle to safety, even though the precise mechanics of this mode may differ.
        The modes supported by a flight stack can be queried using AVAILABLE_MODES and set using MAV_CMD_DO_SET_STANDARD_MODE.
        The current mode is streamed in CURRENT_MODE.
        See https://mavlink.io/en/services/standard_modes.html
       */
enum class MAV_STANDARD_MODE : uint8_t
{
    NON_STANDARD=0, /* Non standard mode.
          This may be used when reporting the mode if the current flight mode is not a standard mode.
         | */
    POSITION_HOLD=1, /* Position mode (manual).
          Position-controlled and stabilized manual mode.
          When sticks are released vehicles return to their level-flight orientation and hold both position and altitude against wind and external forces.
          This mode can only be set by vehicles that can hold a fixed position.
          Multicopter (MC) vehicles actively brake and hold both position and altitude against wind and external forces.
          Hybrid MC/FW ("VTOL") vehicles first transition to multicopter mode (if needed) but otherwise behave in the same way as MC vehicles.
          Fixed-wing (FW) vehicles must not support this mode.
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    ORBIT=2, /* Orbit (manual).
          Position-controlled and stabilized manual mode.
          The vehicle circles around a fixed setpoint in the horizontal plane at a particular radius, altitude, and direction.
          Flight stacks may further allow manual control over the setpoint position, radius, direction, speed, and/or altitude of the circle, but this is not mandated.
          Flight stacks may support the [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) for changing the orbit parameters.
          MC and FW vehicles may support this mode.
          Hybrid MC/FW ("VTOL") vehicles may support this mode in MC/FW or both modes; if the mode is not supported by the current configuration the vehicle should transition to the supported configuration.
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    CRUISE=3, /* Cruise mode (manual).
          Position-controlled and stabilized manual mode.
          When sticks are released vehicles return to their level-flight orientation and hold their original track against wind and external forces.
          Fixed-wing (FW) vehicles level orientation and maintain current track and altitude against wind and external forces.
          Hybrid MC/FW ("VTOL") vehicles first transition to FW mode (if needed) but otherwise behave in the same way as MC vehicles.
          Multicopter (MC) vehicles must not support this mode.
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    ALTITUDE_HOLD=4, /* Altitude hold (manual).
          Altitude-controlled and stabilized manual mode.
          When sticks are released vehicles return to their level-flight orientation and hold their altitude.
          MC vehicles continue with existing momentum and may move with wind (or other external forces).
          FW vehicles continue with current heading, but may be moved off-track by wind.
          Hybrid MC/FW ("VTOL") vehicles behave according to their current configuration/mode (FW or MC).
          Other vehicle types must not support this mode (this may be revisited through the PR process).
         | */
    SAFE_RECOVERY=5, /* Safe recovery mode (auto).
          Automatic mode that takes vehicle to a predefined safe location via a safe flight path, and may also automatically land the vehicle.
          This mode is more commonly referred to as RTL and/or Smart RTL.
          The precise return location, flight path, and landing behaviour depend on vehicle configuration and type.
          For example, the vehicle might return to the home/launch location, a rally point, or the start of a mission landing, it might follow a direct path, mission path, or breadcrumb path, and land using a mission landing pattern or some other kind of descent.
         | */
    MISSION=6, /* Mission mode (automatic).
          Automatic mode that executes MAVLink missions.
          Missions are executed from the current waypoint as soon as the mode is enabled.
         | */
    LAND=7, /* Land mode (auto).
          Automatic mode that lands the vehicle at the current location.
          The precise landing behaviour depends on vehicle configuration and type.
         | */
    TAKEOFF=8, /* Takeoff mode (auto).
          Automatic takeoff mode.
          The precise takeoff behaviour depends on vehicle configuration and type.
         | */
};

//! MAV_STANDARD_MODE ENUM_END
constexpr auto MAV_STANDARD_MODE_ENUM_END = 9;

/** @brief Mode properties.
       */
enum class MAV_MODE_PROPERTY : uint32_t
{
    ADVANCED=1, /* If set, this mode is an advanced mode.
          For example a rate-controlled manual mode might be advanced, whereas a position-controlled manual mode is not.
          A GCS can optionally use this flag to configure the UI for its intended users.
         | */
    NOT_USER_SELECTABLE=2, /* If set, this mode should not be added to the list of selectable modes.
          The mode might still be selected by the FC directly (for example as part of a failsafe).
         | */
    AUTO_MODE=4, /* If set, this mode is automatically controlled (it may use but does not require a manual controller).
          If unset the mode is a assumed to require user input (be a manual mode).
         | */
};

//! MAV_MODE_PROPERTY ENUM_END
constexpr auto MAV_MODE_PROPERTY_ENUM_END = 5;

/** @brief Flags used in HIL_ACTUATOR_CONTROLS message. */
enum class HIL_ACTUATOR_CONTROLS_FLAGS : uint64_t
{
    LOCKSTEP=1, /* Simulation is using lockstep | */
};

//! HIL_ACTUATOR_CONTROLS_FLAGS ENUM_END
constexpr auto HIL_ACTUATOR_CONTROLS_FLAGS_ENUM_END = 2;

/** @brief Flags used to report computer status. */
enum class COMPUTER_STATUS_FLAGS : uint16_t
{
    UNDER_VOLTAGE=1, /* Indicates if the system is experiencing voltage outside of acceptable range. | */
    CPU_THROTTLE=2, /* Indicates if CPU throttling is active. | */
    THERMAL_THROTTLE=4, /* Indicates if thermal throttling is active. | */
    DISK_FULL=8, /* Indicates if main disk is full. | */
};

//! COMPUTER_STATUS_FLAGS ENUM_END
constexpr auto COMPUTER_STATUS_FLAGS_ENUM_END = 9;

/** @brief Airspeed sensor flags */
enum class AIRSPEED_SENSOR_FLAGS : uint8_t
{
    UNHEALTHY=1, /* Airspeed sensor is unhealthy | */
    USING=2, /* True if the data from this sensor is being actively used by the flight controller for guidance, navigation or control. | */
};

//! AIRSPEED_SENSOR_FLAGS ENUM_END
constexpr auto AIRSPEED_SENSOR_FLAGS_ENUM_END = 3;

/** @brief Source for GLOBAL_POSITION measurement or estimate. */
enum class GLOBAL_POSITION_SRC : uint8_t
{
    UNKNOWN=0, /* Source is unknown or not one of the listed types. | */
    GNSS=1, /* Global Navigation Satellite System (e.g.: GPS, Galileo, Glonass, BeiDou). | */
    VISION=2, /* Vision system (e.g.: map matching). | */
    PSEUDOLITES=3, /* A pseudo-satellite system using transceiver beacons to perform GNSS-like positioning. | */
    TERRAIN=4, /* Terrain referenced navigation. | */
    MAGNETIC=5, /* Magnetic positioning. | */
    ESTIMATOR=6, /* Estimated position based on various sensors (eg. a Kalman Filter). | */
    LEO=7, /* Low Earth Orbit satellite-based positioning (e.g.: Starlink, Xona PULSAR). | */
};

//! GLOBAL_POSITION_SRC ENUM_END
constexpr auto GLOBAL_POSITION_SRC_ENUM_END = 8;

/** @brief Status flags for GLOBAL_POSITION */
enum class GLOBAL_POSITION_FLAGS : uint8_t
{
    UNHEALTHY=1, /* Unhealthy sensor/estimator. | */
    PRIMARY=2, /* True if the data originates from or is consumed by the primary estimator. | */
};

//! GLOBAL_POSITION_FLAGS ENUM_END
constexpr auto GLOBAL_POSITION_FLAGS_ENUM_END = 3;

/** @brief Bitmask indicating which fields contain valid data in a FOLLOW_TARGET message (lat/lon/alt, vel, acc, attitude_q, rates). If a bit is unset, the corresponding field(s) are zero-filled and should be ignored. */
enum class FOLLOW_TARGET_CAP_FLAGS : uint8_t
{
    POS=1, /* Position estimate is valid (lat/lon/alt). | */
    VEL=2, /* Velocity estimate is valid (vel field). | */
    ACCEL=4, /* Acceleration estimate is valid (acc field). | */
    ATT_RATES=8, /* Attitude and angular rate estimates are valid (attitude_q and rates fields). | */
};

//! FOLLOW_TARGET_CAP_FLAGS ENUM_END
constexpr auto FOLLOW_TARGET_CAP_FLAGS_ENUM_END = 9;


} // namespace common
} // namespace mavlink

// MESSAGE DEFINITIONS
#include <mavlink/common/mavlink_msg_sys_status.hpp>
#include <mavlink/common/mavlink_msg_system_time.hpp>
#include <mavlink/common/mavlink_msg_ping.hpp>
#include <mavlink/common/mavlink_msg_change_operator_control.hpp>
#include <mavlink/common/mavlink_msg_change_operator_control_ack.hpp>
#include <mavlink/common/mavlink_msg_auth_key.hpp>
#include <mavlink/common/mavlink_msg_link_node_status.hpp>
#include <mavlink/common/mavlink_msg_set_mode.hpp>
#include <mavlink/common/mavlink_msg_param_request_read.hpp>
#include <mavlink/common/mavlink_msg_param_request_list.hpp>
#include <mavlink/common/mavlink_msg_param_value.hpp>
#include <mavlink/common/mavlink_msg_param_set.hpp>
#include <mavlink/common/mavlink_msg_gps_raw_int.hpp>
#include <mavlink/common/mavlink_msg_gps_status.hpp>
#include <mavlink/common/mavlink_msg_scaled_imu.hpp>
#include <mavlink/common/mavlink_msg_raw_imu.hpp>
#include <mavlink/common/mavlink_msg_raw_pressure.hpp>
#include <mavlink/common/mavlink_msg_scaled_pressure.hpp>
#include <mavlink/common/mavlink_msg_attitude.hpp>
#include <mavlink/common/mavlink_msg_attitude_quaternion.hpp>
#include <mavlink/common/mavlink_msg_local_position_ned.hpp>
#include <mavlink/common/mavlink_msg_rc_channels_scaled.hpp>
#include <mavlink/common/mavlink_msg_rc_channels_raw.hpp>
#include <mavlink/common/mavlink_msg_servo_output_raw.hpp>
#include <mavlink/common/mavlink_msg_mission_request_partial_list.hpp>
#include <mavlink/common/mavlink_msg_mission_write_partial_list.hpp>
#include <mavlink/common/mavlink_msg_mission_item.hpp>
#include <mavlink/common/mavlink_msg_mission_request.hpp>
#include <mavlink/common/mavlink_msg_mission_set_current.hpp>
#include <mavlink/common/mavlink_msg_mission_current.hpp>
#include <mavlink/common/mavlink_msg_mission_request_list.hpp>
#include <mavlink/common/mavlink_msg_mission_count.hpp>
#include <mavlink/common/mavlink_msg_mission_clear_all.hpp>
#include <mavlink/common/mavlink_msg_mission_item_reached.hpp>
#include <mavlink/common/mavlink_msg_mission_ack.hpp>
#include <mavlink/common/mavlink_msg_set_gps_global_origin.hpp>
#include <mavlink/common/mavlink_msg_gps_global_origin.hpp>
#include <mavlink/common/mavlink_msg_param_map_rc.hpp>
#include <mavlink/common/mavlink_msg_mission_request_int.hpp>
#include <mavlink/common/mavlink_msg_safety_set_allowed_area.hpp>
#include <mavlink/common/mavlink_msg_safety_allowed_area.hpp>
#include <mavlink/common/mavlink_msg_attitude_quaternion_cov.hpp>
#include <mavlink/common/mavlink_msg_nav_controller_output.hpp>
#include <mavlink/common/mavlink_msg_global_position_int_cov.hpp>
#include <mavlink/common/mavlink_msg_local_position_ned_cov.hpp>
#include <mavlink/common/mavlink_msg_rc_channels.hpp>
#include <mavlink/common/mavlink_msg_request_data_stream.hpp>
#include <mavlink/common/mavlink_msg_data_stream.hpp>
#include <mavlink/common/mavlink_msg_manual_control.hpp>
#include <mavlink/common/mavlink_msg_rc_channels_override.hpp>
#include <mavlink/common/mavlink_msg_mission_item_int.hpp>
#include <mavlink/common/mavlink_msg_vfr_hud.hpp>
#include <mavlink/common/mavlink_msg_command_int.hpp>
#include <mavlink/common/mavlink_msg_command_long.hpp>
#include <mavlink/common/mavlink_msg_command_ack.hpp>
#include <mavlink/common/mavlink_msg_command_cancel.hpp>
#include <mavlink/common/mavlink_msg_manual_setpoint.hpp>
#include <mavlink/common/mavlink_msg_set_attitude_target.hpp>
#include <mavlink/common/mavlink_msg_attitude_target.hpp>
#include <mavlink/common/mavlink_msg_set_position_target_local_ned.hpp>
#include <mavlink/common/mavlink_msg_position_target_local_ned.hpp>
#include <mavlink/common/mavlink_msg_set_position_target_global_int.hpp>
#include <mavlink/common/mavlink_msg_position_target_global_int.hpp>
#include <mavlink/common/mavlink_msg_local_position_ned_system_global_offset.hpp>
#include <mavlink/common/mavlink_msg_hil_state.hpp>
#include <mavlink/common/mavlink_msg_hil_controls.hpp>
#include <mavlink/common/mavlink_msg_hil_rc_inputs_raw.hpp>
#include <mavlink/common/mavlink_msg_hil_actuator_controls.hpp>
#include <mavlink/common/mavlink_msg_optical_flow.hpp>
#include <mavlink/common/mavlink_msg_global_vision_position_estimate.hpp>
#include <mavlink/common/mavlink_msg_vision_position_estimate.hpp>
#include <mavlink/common/mavlink_msg_vision_speed_estimate.hpp>
#include <mavlink/common/mavlink_msg_vicon_position_estimate.hpp>
#include <mavlink/common/mavlink_msg_highres_imu.hpp>
#include <mavlink/common/mavlink_msg_optical_flow_rad.hpp>
#include <mavlink/common/mavlink_msg_hil_sensor.hpp>
#include <mavlink/common/mavlink_msg_sim_state.hpp>
#include <mavlink/common/mavlink_msg_radio_status.hpp>
#include <mavlink/common/mavlink_msg_file_transfer_protocol.hpp>
#include <mavlink/common/mavlink_msg_timesync.hpp>
#include <mavlink/common/mavlink_msg_camera_trigger.hpp>
#include <mavlink/common/mavlink_msg_hil_gps.hpp>
#include <mavlink/common/mavlink_msg_hil_optical_flow.hpp>
#include <mavlink/common/mavlink_msg_hil_state_quaternion.hpp>
#include <mavlink/common/mavlink_msg_scaled_imu2.hpp>
#include <mavlink/common/mavlink_msg_log_request_list.hpp>
#include <mavlink/common/mavlink_msg_log_entry.hpp>
#include <mavlink/common/mavlink_msg_log_request_data.hpp>
#include <mavlink/common/mavlink_msg_log_data.hpp>
#include <mavlink/common/mavlink_msg_log_erase.hpp>
#include <mavlink/common/mavlink_msg_log_request_end.hpp>
#include <mavlink/common/mavlink_msg_gps_inject_data.hpp>
#include <mavlink/common/mavlink_msg_gps2_raw.hpp>
#include <mavlink/common/mavlink_msg_power_status.hpp>
#include <mavlink/common/mavlink_msg_serial_control.hpp>
#include <mavlink/common/mavlink_msg_gps_rtk.hpp>
#include <mavlink/common/mavlink_msg_gps2_rtk.hpp>
#include <mavlink/common/mavlink_msg_scaled_imu3.hpp>
#include <mavlink/common/mavlink_msg_data_transmission_handshake.hpp>
#include <mavlink/common/mavlink_msg_encapsulated_data.hpp>
#include <mavlink/common/mavlink_msg_distance_sensor.hpp>
#include <mavlink/common/mavlink_msg_terrain_request.hpp>
#include <mavlink/common/mavlink_msg_terrain_data.hpp>
#include <mavlink/common/mavlink_msg_terrain_check.hpp>
#include <mavlink/common/mavlink_msg_terrain_report.hpp>
#include <mavlink/common/mavlink_msg_scaled_pressure2.hpp>
#include <mavlink/common/mavlink_msg_att_pos_mocap.hpp>
#include <mavlink/common/mavlink_msg_set_actuator_control_target.hpp>
#include <mavlink/common/mavlink_msg_actuator_control_target.hpp>
#include <mavlink/common/mavlink_msg_altitude.hpp>
#include <mavlink/common/mavlink_msg_resource_request.hpp>
#include <mavlink/common/mavlink_msg_scaled_pressure3.hpp>
#include <mavlink/common/mavlink_msg_follow_target.hpp>
#include <mavlink/common/mavlink_msg_control_system_state.hpp>
#include <mavlink/common/mavlink_msg_battery_status.hpp>
#include <mavlink/common/mavlink_msg_landing_target.hpp>
#include <mavlink/common/mavlink_msg_fence_status.hpp>
#include <mavlink/common/mavlink_msg_mag_cal_report.hpp>
#include <mavlink/common/mavlink_msg_efi_status.hpp>
#include <mavlink/common/mavlink_msg_estimator_status.hpp>
#include <mavlink/common/mavlink_msg_wind_cov.hpp>
#include <mavlink/common/mavlink_msg_gps_input.hpp>
#include <mavlink/common/mavlink_msg_gps_rtcm_data.hpp>
#include <mavlink/common/mavlink_msg_high_latency.hpp>
#include <mavlink/common/mavlink_msg_high_latency2.hpp>
#include <mavlink/common/mavlink_msg_vibration.hpp>
#include <mavlink/common/mavlink_msg_home_position.hpp>
#include <mavlink/common/mavlink_msg_set_home_position.hpp>
#include <mavlink/common/mavlink_msg_message_interval.hpp>
#include <mavlink/common/mavlink_msg_extended_sys_state.hpp>
#include <mavlink/common/mavlink_msg_adsb_vehicle.hpp>
#include <mavlink/common/mavlink_msg_collision.hpp>
#include <mavlink/common/mavlink_msg_v2_extension.hpp>
#include <mavlink/common/mavlink_msg_memory_vect.hpp>
#include <mavlink/common/mavlink_msg_debug_vect.hpp>
#include <mavlink/common/mavlink_msg_named_value_float.hpp>
#include <mavlink/common/mavlink_msg_named_value_int.hpp>
#include <mavlink/common/mavlink_msg_statustext.hpp>
#include <mavlink/common/mavlink_msg_debug.hpp>
#include <mavlink/common/mavlink_msg_setup_signing.hpp>
#include <mavlink/common/mavlink_msg_button_change.hpp>
#include <mavlink/common/mavlink_msg_play_tune.hpp>
#include <mavlink/common/mavlink_msg_camera_information.hpp>
#include <mavlink/common/mavlink_msg_camera_settings.hpp>
#include <mavlink/common/mavlink_msg_storage_information.hpp>
#include <mavlink/common/mavlink_msg_camera_capture_status.hpp>
#include <mavlink/common/mavlink_msg_camera_image_captured.hpp>
#include <mavlink/common/mavlink_msg_flight_information.hpp>
#include <mavlink/common/mavlink_msg_mount_orientation.hpp>
#include <mavlink/common/mavlink_msg_logging_data.hpp>
#include <mavlink/common/mavlink_msg_logging_data_acked.hpp>
#include <mavlink/common/mavlink_msg_logging_ack.hpp>
#include <mavlink/common/mavlink_msg_video_stream_information.hpp>
#include <mavlink/common/mavlink_msg_video_stream_status.hpp>
#include <mavlink/common/mavlink_msg_camera_fov_status.hpp>
#include <mavlink/common/mavlink_msg_camera_tracking_image_status.hpp>
#include <mavlink/common/mavlink_msg_camera_tracking_geo_status.hpp>
#include <mavlink/common/mavlink_msg_camera_thermal_range.hpp>
#include <mavlink/common/mavlink_msg_gimbal_manager_information.hpp>
#include <mavlink/common/mavlink_msg_gimbal_manager_status.hpp>
#include <mavlink/common/mavlink_msg_gimbal_manager_set_attitude.hpp>
#include <mavlink/common/mavlink_msg_gimbal_device_information.hpp>
#include <mavlink/common/mavlink_msg_gimbal_device_set_attitude.hpp>
#include <mavlink/common/mavlink_msg_gimbal_device_attitude_status.hpp>
#include <mavlink/common/mavlink_msg_autopilot_state_for_gimbal_device.hpp>
#include <mavlink/common/mavlink_msg_gimbal_manager_set_pitchyaw.hpp>
#include <mavlink/common/mavlink_msg_gimbal_manager_set_manual_control.hpp>
#include <mavlink/common/mavlink_msg_esc_info.hpp>
#include <mavlink/common/mavlink_msg_esc_status.hpp>
#include <mavlink/common/mavlink_msg_airspeed.hpp>
#include <mavlink/common/mavlink_msg_global_position_sensor.hpp>
#include <mavlink/common/mavlink_msg_wifi_config_ap.hpp>
#include <mavlink/common/mavlink_msg_protocol_version.hpp>
#include <mavlink/common/mavlink_msg_ais_vessel.hpp>
#include <mavlink/common/mavlink_msg_uavcan_node_status.hpp>
#include <mavlink/common/mavlink_msg_uavcan_node_info.hpp>
#include <mavlink/common/mavlink_msg_param_ext_request_read.hpp>
#include <mavlink/common/mavlink_msg_param_ext_request_list.hpp>
#include <mavlink/common/mavlink_msg_param_ext_value.hpp>
#include <mavlink/common/mavlink_msg_param_ext_set.hpp>
#include <mavlink/common/mavlink_msg_param_ext_ack.hpp>
#include <mavlink/common/mavlink_msg_obstacle_distance.hpp>
#include <mavlink/common/mavlink_msg_odometry.hpp>
#include <mavlink/common/mavlink_msg_trajectory_representation_waypoints.hpp>
#include <mavlink/common/mavlink_msg_trajectory_representation_bezier.hpp>
#include <mavlink/common/mavlink_msg_cellular_status.hpp>
#include <mavlink/common/mavlink_msg_isbd_link_status.hpp>
#include <mavlink/common/mavlink_msg_cellular_config.hpp>
#include <mavlink/common/mavlink_msg_raw_rpm.hpp>
#include <mavlink/common/mavlink_msg_utm_global_position.hpp>
#include <mavlink/common/mavlink_msg_param_error.hpp>
#include <mavlink/common/mavlink_msg_debug_float_array.hpp>
#include <mavlink/common/mavlink_msg_orbit_execution_status.hpp>
#include <mavlink/common/mavlink_msg_smart_battery_info.hpp>
#include <mavlink/common/mavlink_msg_figure_eight_execution_status.hpp>
#include <mavlink/common/mavlink_msg_fuel_status.hpp>
#include <mavlink/common/mavlink_msg_battery_info.hpp>
#include <mavlink/common/mavlink_msg_generator_status.hpp>
#include <mavlink/common/mavlink_msg_actuator_output_status.hpp>
#include <mavlink/common/mavlink_msg_relay_status.hpp>
#include <mavlink/common/mavlink_msg_time_estimate_to_target.hpp>
#include <mavlink/common/mavlink_msg_tunnel.hpp>
#include <mavlink/common/mavlink_msg_can_frame.hpp>
#include <mavlink/common/mavlink_msg_canfd_frame.hpp>
#include <mavlink/common/mavlink_msg_can_filter_modify.hpp>
#include <mavlink/common/mavlink_msg_onboard_computer_status.hpp>
#include <mavlink/common/mavlink_msg_component_information.hpp>
#include <mavlink/common/mavlink_msg_component_information_basic.hpp>
#include <mavlink/common/mavlink_msg_component_metadata.hpp>
#include <mavlink/common/mavlink_msg_play_tune_v2.hpp>
#include <mavlink/common/mavlink_msg_supported_tunes.hpp>
#include <mavlink/common/mavlink_msg_event.hpp>
#include <mavlink/common/mavlink_msg_current_event_sequence.hpp>
#include <mavlink/common/mavlink_msg_request_event.hpp>
#include <mavlink/common/mavlink_msg_response_event_error.hpp>
#include <mavlink/common/mavlink_msg_radio_rc_channels.hpp>
#include <mavlink/common/mavlink_msg_available_modes.hpp>
#include <mavlink/common/mavlink_msg_current_mode.hpp>
#include <mavlink/common/mavlink_msg_available_modes_monitor.hpp>
#include <mavlink/common/mavlink_msg_illuminator_status.hpp>
#include <mavlink/common/mavlink_msg_wheel_distance.hpp>
#include <mavlink/common/mavlink_msg_winch_status.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_basic_id.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_location.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_authentication.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_self_id.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_system.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_operator_id.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_message_pack.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_arm_status.hpp>
#include <mavlink/common/mavlink_msg_open_drone_id_system_update.hpp>
#include <mavlink/common/mavlink_msg_hygrometer_sensor.hpp>

// base include
#include <mavlink/standard/standard.hpp>
