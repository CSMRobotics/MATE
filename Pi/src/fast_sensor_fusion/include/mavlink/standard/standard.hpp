/** @file
 *  @brief MAVLink comm protocol generated from standard.xml
 *  @see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace standard {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 3> MESSAGE_ENTRIES {{ {0, 50, 9, 9, 0, 0, 0}, {33, 104, 28, 28, 0, 0, 0}, {148, 178, 60, 78, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 2;


// ENUM DEFINITIONS


/** @brief Enum used to indicate true or false (also: success or failure, enabled or disabled, active or inactive). */
enum class MAV_BOOL
{
    FALSE_=0, /* False. | */
    TRUE_=1, /* True. | */
};

//! MAV_BOOL ENUM_END
constexpr auto MAV_BOOL_ENUM_END = 2;

/** @brief Bitmask of (optional) autopilot capabilities (64 bit). If a bit is set, the autopilot supports this capability. */
enum class MAV_PROTOCOL_CAPABILITY : uint64_t
{
    MISSION_FLOAT=1, /* Autopilot supports the MISSION_ITEM float message type.
          Note that MISSION_ITEM is deprecated, and autopilots should use MISSION_ITEM_INT instead.
         | */
    PARAM_FLOAT=2, /* Autopilot supports the new param float message type. | */
    MISSION_INT=4, /* Autopilot supports MISSION_ITEM_INT scaled integer message type.
          Note that this flag must always be set if missions are supported, because missions must always use MISSION_ITEM_INT (rather than MISSION_ITEM, which is deprecated).
         | */
    COMMAND_INT=8, /* Autopilot supports COMMAND_INT scaled integer message type. | */
    PARAM_ENCODE_BYTEWISE=16, /* Parameter protocol uses byte-wise encoding of parameter values into param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.
          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST should be set if the parameter protocol is supported.
         | */
    FTP=32, /* Autopilot supports the File Transfer Protocol v1: https://mavlink.io/en/services/ftp.html. | */
    SET_ATTITUDE_TARGET=64, /* Autopilot supports the SET_ATTITUDE_TARGET message (for commanding attitude from an offboard controller). | */
    SET_POSITION_TARGET_LOCAL_NED=128, /* Autopilot supports the SET_POSITION_TARGET_LOCAL_NED message (for commanding position and velocity targets in local NED frame). | */
    SET_POSITION_TARGET_GLOBAL_INT=256, /* Autopilot supports the SET_POSITION_TARGET_GLOBAL_INT message (for commanding position and velocity targets in global scaled integers). | */
    TERRAIN=512, /* Autopilot supports terrain protocol / data handling. | */
    RESERVED3=1024, /* Reserved for future use. | */
    FLIGHT_TERMINATION=2048, /* Autopilot supports the MAV_CMD_DO_FLIGHTTERMINATION command (flight termination). | */
    COMPASS_CALIBRATION=4096, /* Autopilot supports onboard compass calibration. | */
    MAVLINK2=8192, /* Autopilot supports MAVLink version 2. | */
    MISSION_FENCE=16384, /* Autopilot supports mission fence protocol. | */
    MISSION_RALLY=32768, /* Autopilot supports mission rally point protocol. | */
    RESERVED2=65536, /* Reserved for future use. | */
    PARAM_ENCODE_C_CAST=131072, /* Parameter protocol uses C-cast of parameter values to set the param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.
          Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE should be set if the parameter protocol is supported.
         | */
    COMPONENT_IMPLEMENTS_GIMBAL_MANAGER=262144, /* This component implements/is a gimbal manager. This means the GIMBAL_MANAGER_INFORMATION, and other messages can be requested.
         | */
    COMPONENT_ACCEPTS_GCS_CONTROL=524288, /* Component supports locking control to a particular GCS independent of its system (via MAV_CMD_REQUEST_OPERATOR_CONTROL). | */
    GRIPPER=1048576, /* Autopilot has a connected gripper. MAVLink Grippers would set MAV_TYPE_GRIPPER instead. | */
};

//! MAV_PROTOCOL_CAPABILITY ENUM_END
constexpr auto MAV_PROTOCOL_CAPABILITY_ENUM_END = 1048577;

/** @brief These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65. */
enum class FIRMWARE_VERSION_TYPE
{
    DEV=0, /* development release | */
    ALPHA=64, /* alpha release | */
    BETA=128, /* beta release | */
    RC=192, /* release candidate | */
    OFFICIAL=255, /* official stable release | */
};

//! FIRMWARE_VERSION_TYPE ENUM_END
constexpr auto FIRMWARE_VERSION_TYPE_ENUM_END = 256;


} // namespace standard
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_global_position_int.hpp"
#include "./mavlink_msg_autopilot_version.hpp"

// base include
#include "../minimal/minimal.hpp"
