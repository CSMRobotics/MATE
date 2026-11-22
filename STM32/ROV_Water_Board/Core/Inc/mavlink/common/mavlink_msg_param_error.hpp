// MESSAGE PARAM_ERROR support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief PARAM_ERROR message
 *
 * Parameter set/get error. Returned from a MAVLink node in response to an error in the parameter protocol, for example failing to set a parameter because it does not exist.
      
 */
struct PARAM_ERROR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 345;
    static constexpr size_t LENGTH = 21;
    static constexpr size_t MIN_LENGTH = 21;
    static constexpr uint8_t CRC_EXTRA = 209;
    static constexpr auto NAME = "PARAM_ERROR";


    uint8_t target_system; /*<  System ID */
    uint8_t target_component; /*<  Component ID */
    std::array<char, 16> param_id; /*<  Parameter id. Terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string */
    int16_t param_index; /*<  Parameter index. Will be -1 if the param ID field should be used as an identifier (else the param id will be ignored) */
    uint8_t error; /*<  Error being returned to client. */


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
        ss << "  target_system: " << +target_system << std::endl;
        ss << "  target_component: " << +target_component << std::endl;
        ss << "  param_id: \"" << to_string(param_id) << "\"" << std::endl;
        ss << "  param_index: " << param_index << std::endl;
        ss << "  error: " << +error << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << param_index;                   // offset: 0
        map << target_system;                 // offset: 2
        map << target_component;              // offset: 3
        map << param_id;                      // offset: 4
        map << error;                         // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> param_index;                   // offset: 0
        map >> target_system;                 // offset: 2
        map >> target_component;              // offset: 3
        map >> param_id;                      // offset: 4
        map >> error;                         // offset: 20
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
