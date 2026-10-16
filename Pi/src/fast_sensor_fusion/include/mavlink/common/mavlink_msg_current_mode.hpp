// MESSAGE CURRENT_MODE support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CURRENT_MODE message
 *
 * Get the current mode.
        This should be emitted on any mode change, and broadcast at low rate (nominally 0.5 Hz).
        It may be requested using MAV_CMD_REQUEST_MESSAGE.
        See https://mavlink.io/en/services/standard_modes.html
      
 */
struct CURRENT_MODE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 436;
    static constexpr size_t LENGTH = 9;
    static constexpr size_t MIN_LENGTH = 9;
    static constexpr uint8_t CRC_EXTRA = 193;
    static constexpr auto NAME = "CURRENT_MODE";


    uint8_t standard_mode; /*<  Standard mode. */
    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags */
    uint32_t intended_custom_mode; /*<  The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied */


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
        ss << "  standard_mode: " << +standard_mode << std::endl;
        ss << "  custom_mode: " << custom_mode << std::endl;
        ss << "  intended_custom_mode: " << intended_custom_mode << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << custom_mode;                   // offset: 0
        map << intended_custom_mode;          // offset: 4
        map << standard_mode;                 // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> custom_mode;                   // offset: 0
        map >> intended_custom_mode;          // offset: 4
        map >> standard_mode;                 // offset: 8
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
