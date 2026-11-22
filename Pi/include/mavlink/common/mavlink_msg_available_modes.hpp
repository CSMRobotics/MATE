// MESSAGE AVAILABLE_MODES support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief AVAILABLE_MODES message
 *
 * Information about a flight mode.

        The message can be enumerated to get information for all modes, or requested for a particular mode, using MAV_CMD_REQUEST_MESSAGE.
        Specify 0 in param2 to request that the message is emitted for all available modes or the specific index for just one mode.
        The modes must be available/settable for the current vehicle/frame type.
        Each mode should only be emitted once (even if it is both standard and custom).
        Note that the current mode should be emitted in CURRENT_MODE, and that if the mode list can change then AVAILABLE_MODES_MONITOR must be emitted on first change and subsequently streamed.
        See https://mavlink.io/en/services/standard_modes.html
      
 */
struct AVAILABLE_MODES : mavlink::Message {
    static constexpr msgid_t MSG_ID = 435;
    static constexpr size_t LENGTH = 47;
    static constexpr size_t MIN_LENGTH = 46;
    static constexpr uint8_t CRC_EXTRA = 134;
    static constexpr auto NAME = "AVAILABLE_MODES";


    uint8_t number_modes; /*<  The total number of available modes for the current vehicle type. */
    uint8_t mode_index; /*<  The current mode index within number_modes, indexed from 1. The index is not guaranteed to be persistent, and may change between reboots or if the set of modes change. */
    uint8_t standard_mode; /*<  Standard mode. */
    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags */
    uint32_t properties; /*<  Mode properties. */
    std::array<char, 35> mode_name; /*<  Name of custom mode, with null termination character. Should be omitted for standard modes. */
    uint8_t seq; /*<  Sequence number. The value iterates sequentially whenever the set of AVAILABLE_MODES changes and should match value of AVAILABLE_MODES_MONITOR. Note, a GCS must ignore 0 values, and should re-start the download if the value changes part-way through fetching modes. */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  number_modes: " << +number_modes << std::endl;
        ss << "  mode_index: " << +mode_index << std::endl;
        ss << "  standard_mode: " << +standard_mode << std::endl;
        ss << "  custom_mode: " << custom_mode << std::endl;
        ss << "  properties: " << properties << std::endl;
        ss << "  mode_name: \"" << to_string(mode_name) << "\"" << std::endl;
        ss << "  seq: " << +seq << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << custom_mode;                   // offset: 0
        map << properties;                    // offset: 4
        map << number_modes;                  // offset: 8
        map << mode_index;                    // offset: 9
        map << standard_mode;                 // offset: 10
        map << mode_name;                     // offset: 11
        map << seq;                           // offset: 46
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> custom_mode;                   // offset: 0
        map >> properties;                    // offset: 4
        map >> number_modes;                  // offset: 8
        map >> mode_index;                    // offset: 9
        map >> standard_mode;                 // offset: 10
        map >> mode_name;                     // offset: 11
        map >> seq;                           // offset: 46
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
