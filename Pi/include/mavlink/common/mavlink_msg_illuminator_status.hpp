// MESSAGE ILLUMINATOR_STATUS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief ILLUMINATOR_STATUS message
 *
 * Illuminator status
 */
struct ILLUMINATOR_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 440;
    static constexpr size_t LENGTH = 35;
    static constexpr size_t MIN_LENGTH = 35;
    static constexpr uint8_t CRC_EXTRA = 66;
    static constexpr auto NAME = "ILLUMINATOR_STATUS";


    uint32_t uptime_ms; /*< [ms] Time since the start-up of the illuminator in ms */
    uint8_t enable; /*<  0: Illuminators OFF, 1: Illuminators ON */
    uint8_t mode_bitmask; /*<  Supported illuminator modes */
    uint32_t error_status; /*<  Errors */
    uint8_t mode; /*<  Illuminator mode */
    float brightness; /*< [%] Illuminator brightness */
    float strobe_period; /*< [s] Illuminator strobing period in seconds */
    float strobe_duty_cycle; /*< [%] Illuminator strobing duty cycle */
    float temp_c; /*<  Temperature in Celsius */
    float min_strobe_period; /*< [s] Minimum strobing period in seconds */
    float max_strobe_period; /*< [s] Maximum strobing period in seconds */


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
        ss << "  uptime_ms: " << uptime_ms << std::endl;
        ss << "  enable: " << +enable << std::endl;
        ss << "  mode_bitmask: " << +mode_bitmask << std::endl;
        ss << "  error_status: " << error_status << std::endl;
        ss << "  mode: " << +mode << std::endl;
        ss << "  brightness: " << brightness << std::endl;
        ss << "  strobe_period: " << strobe_period << std::endl;
        ss << "  strobe_duty_cycle: " << strobe_duty_cycle << std::endl;
        ss << "  temp_c: " << temp_c << std::endl;
        ss << "  min_strobe_period: " << min_strobe_period << std::endl;
        ss << "  max_strobe_period: " << max_strobe_period << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << uptime_ms;                     // offset: 0
        map << error_status;                  // offset: 4
        map << brightness;                    // offset: 8
        map << strobe_period;                 // offset: 12
        map << strobe_duty_cycle;             // offset: 16
        map << temp_c;                        // offset: 20
        map << min_strobe_period;             // offset: 24
        map << max_strobe_period;             // offset: 28
        map << enable;                        // offset: 32
        map << mode_bitmask;                  // offset: 33
        map << mode;                          // offset: 34
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> uptime_ms;                     // offset: 0
        map >> error_status;                  // offset: 4
        map >> brightness;                    // offset: 8
        map >> strobe_period;                 // offset: 12
        map >> strobe_duty_cycle;             // offset: 16
        map >> temp_c;                        // offset: 20
        map >> min_strobe_period;             // offset: 24
        map >> max_strobe_period;             // offset: 28
        map >> enable;                        // offset: 32
        map >> mode_bitmask;                  // offset: 33
        map >> mode;                          // offset: 34
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
