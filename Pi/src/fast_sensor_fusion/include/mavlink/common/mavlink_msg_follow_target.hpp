// MESSAGE FOLLOW_TARGET support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief FOLLOW_TARGET message
 *
 * Current motion information from a designated system
 */
struct FOLLOW_TARGET : mavlink::Message {
    static constexpr msgid_t MSG_ID = 144;
    static constexpr size_t LENGTH = 93;
    static constexpr size_t MIN_LENGTH = 93;
    static constexpr uint8_t CRC_EXTRA = 127;
    static constexpr auto NAME = "FOLLOW_TARGET";


    uint64_t timestamp; /*< [ms] Timestamp (time since system boot). */
    uint8_t est_capabilities; /*<  Bitmask indicating which fields in this message contain valid data. */
    int32_t lat; /*< [degE7] Latitude (WGS84) */
    int32_t lon; /*< [degE7] Longitude (WGS84) */
    float alt; /*< [m] Altitude (MSL) */
    std::array<float, 3> vel; /*< [m/s] Target velocity in MAV_FRAME_LOCAL_NED frame. (0,0,0) for unknown. */
    std::array<float, 3> acc; /*< [m/s/s] Target linear acceleration in MAV_FRAME_LOCAL_NED frame. (0,0,0) for unknown. */
    std::array<float, 4> attitude_q; /*<  Target orientation as a quaternion rotating from MAV_FRAME_BODY_FRD to MAV_FRAME_LOCAL_NED (w, x, y, z order, zero-rotation is [1, 0, 0, 0]). (0, 0, 0, 0) for unknown. */
    std::array<float, 3> rates; /*< [rad/s] Target angular rates (roll, pitch, yaw) in MAV_FRAME_BODY_FRD frame. (0,0,0) for unknown. */
    std::array<float, 3> position_cov; /*<  eph epv */
    uint64_t custom_state; /*<  button states or switches of a tracker device */


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
        ss << "  timestamp: " << timestamp << std::endl;
        ss << "  est_capabilities: " << +est_capabilities << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  vel: [" << to_string(vel) << "]" << std::endl;
        ss << "  acc: [" << to_string(acc) << "]" << std::endl;
        ss << "  attitude_q: [" << to_string(attitude_q) << "]" << std::endl;
        ss << "  rates: [" << to_string(rates) << "]" << std::endl;
        ss << "  position_cov: [" << to_string(position_cov) << "]" << std::endl;
        ss << "  custom_state: " << custom_state << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << timestamp;                     // offset: 0
        map << custom_state;                  // offset: 8
        map << lat;                           // offset: 16
        map << lon;                           // offset: 20
        map << alt;                           // offset: 24
        map << vel;                           // offset: 28
        map << acc;                           // offset: 40
        map << attitude_q;                    // offset: 52
        map << rates;                         // offset: 68
        map << position_cov;                  // offset: 80
        map << est_capabilities;              // offset: 92
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> timestamp;                     // offset: 0
        map >> custom_state;                  // offset: 8
        map >> lat;                           // offset: 16
        map >> lon;                           // offset: 20
        map >> alt;                           // offset: 24
        map >> vel;                           // offset: 28
        map >> acc;                           // offset: 40
        map >> attitude_q;                    // offset: 52
        map >> rates;                         // offset: 68
        map >> position_cov;                  // offset: 80
        map >> est_capabilities;              // offset: 92
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
