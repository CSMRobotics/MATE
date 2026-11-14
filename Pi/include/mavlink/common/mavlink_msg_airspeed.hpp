// MESSAGE AIRSPEED support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief AIRSPEED message
 *
 * Airspeed information from a sensor.
 */
struct AIRSPEED : mavlink::Message {
    static constexpr msgid_t MSG_ID = 295;
    static constexpr size_t LENGTH = 12;
    static constexpr size_t MIN_LENGTH = 12;
    static constexpr uint8_t CRC_EXTRA = 234;
    static constexpr auto NAME = "AIRSPEED";


    uint8_t id; /*<  Sensor ID. */
    float airspeed; /*< [m/s] Calibrated airspeed (CAS). */
    int16_t temperature; /*< [cdegC] Temperature. */
    float raw_press; /*< [hPa] Raw differential pressure. */
    uint8_t flags; /*<  Airspeed sensor flags. */


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
        ss << "  id: " << +id << std::endl;
        ss << "  airspeed: " << airspeed << std::endl;
        ss << "  temperature: " << temperature << std::endl;
        ss << "  raw_press: " << raw_press << std::endl;
        ss << "  flags: " << +flags << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << airspeed;                      // offset: 0
        map << raw_press;                     // offset: 4
        map << temperature;                   // offset: 8
        map << id;                            // offset: 10
        map << flags;                         // offset: 11
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> airspeed;                      // offset: 0
        map >> raw_press;                     // offset: 4
        map >> temperature;                   // offset: 8
        map >> id;                            // offset: 10
        map >> flags;                         // offset: 11
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
