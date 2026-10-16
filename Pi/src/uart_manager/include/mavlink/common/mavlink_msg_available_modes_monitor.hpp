// MESSAGE AVAILABLE_MODES_MONITOR support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief AVAILABLE_MODES_MONITOR message
 *
 * A change to the sequence number indicates that the set of AVAILABLE_MODES has changed, and that the receiver should re-request all available modes.

        The message is optional, and is only needed when the set of modes can change dynamically after boot.
        It should be emitted whenever the set of modes change.
        It should be streamed at low rate (nominally 0.3 Hz).
        See https://mavlink.io/en/services/standard_modes.html
      
 */
struct AVAILABLE_MODES_MONITOR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 437;
    static constexpr size_t LENGTH = 1;
    static constexpr size_t MIN_LENGTH = 1;
    static constexpr uint8_t CRC_EXTRA = 30;
    static constexpr auto NAME = "AVAILABLE_MODES_MONITOR";


    uint8_t seq; /*<  Sequence number. Iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically). 0 initially. 1 on first change of mode set. */


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
        ss << "  seq: " << +seq << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << seq;                           // offset: 0
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> seq;                           // offset: 0
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
