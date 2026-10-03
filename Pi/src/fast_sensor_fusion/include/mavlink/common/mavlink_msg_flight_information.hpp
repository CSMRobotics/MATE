// MESSAGE FLIGHT_INFORMATION support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief FLIGHT_INFORMATION message
 *
 * Flight information.
        This includes time since boot for arm, takeoff, and land, and a flight number.
        Takeoff and landing values reset to zero on arm.
        This can be requested using MAV_CMD_REQUEST_MESSAGE.
        Note, some fields are misnamed - timestamps are from boot (not UTC) and the flight_uuid is a sequence number.
      
 */
struct FLIGHT_INFORMATION : mavlink::Message {
    static constexpr msgid_t MSG_ID = 264;
    static constexpr size_t LENGTH = 32;
    static constexpr size_t MIN_LENGTH = 28;
    static constexpr uint8_t CRC_EXTRA = 49;
    static constexpr auto NAME = "FLIGHT_INFORMATION";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    uint64_t arming_time_utc; /*< [us] Timestamp at arming (since system boot). Set to 0 on boot. Set value on arming. Note, field is misnamed UTC. */
    uint64_t takeoff_time_utc; /*< [us] Timestamp at takeoff (since system boot). Set to 0 at boot and on arming. Note, field is misnamed UTC. */
    uint64_t flight_uuid; /*<  Flight number. Note, field is misnamed UUID. */
    uint32_t landing_time; /*< [ms] Timestamp at landing (in ms since system boot). Set to 0 at boot and on arming. */


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
        ss << "  time_boot_ms: " << time_boot_ms << std::endl;
        ss << "  arming_time_utc: " << arming_time_utc << std::endl;
        ss << "  takeoff_time_utc: " << takeoff_time_utc << std::endl;
        ss << "  flight_uuid: " << flight_uuid << std::endl;
        ss << "  landing_time: " << landing_time << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << arming_time_utc;               // offset: 0
        map << takeoff_time_utc;              // offset: 8
        map << flight_uuid;                   // offset: 16
        map << time_boot_ms;                  // offset: 24
        map << landing_time;                  // offset: 28
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> arming_time_utc;               // offset: 0
        map >> takeoff_time_utc;              // offset: 8
        map >> flight_uuid;                   // offset: 16
        map >> time_boot_ms;                  // offset: 24
        map >> landing_time;                  // offset: 28
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
