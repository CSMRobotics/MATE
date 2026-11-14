// MESSAGE GLOBAL_POSITION_SENSOR support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief GLOBAL_POSITION_SENSOR message
 *
 * Reports measurement/estimate from a global position sensor. Used as navigation fusion source and optionally displayed in the UI.
 */
struct GLOBAL_POSITION_SENSOR : mavlink::Message {
    static constexpr msgid_t MSG_ID = 296;
    static constexpr size_t LENGTH = 41;
    static constexpr size_t MIN_LENGTH = 41;
    static constexpr uint8_t CRC_EXTRA = 158;
    static constexpr auto NAME = "GLOBAL_POSITION_SENSOR";


    uint8_t target_system; /*<  System ID (ID of target system, normally autopilot and ground station). */
    uint8_t target_component; /*<  Component ID (normally 0 for broadcast). */
    uint8_t id; /*<  Sensor ID */
    uint64_t time_usec; /*< [us] Timestamp of message transmission (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. */
    uint32_t processing_time; /*< [us] The time spent in processing the sensor data that is the basis for this position. The recipient can use this to improve time alignment of the data. This is the time between measurement (e.g. camera exposure time) and transmission of this message. Set to NaN if not known. */
    uint8_t source; /*<  Source of position/estimate (such as GNSS, estimator, etc.) */
    uint8_t flags; /*<  Status flags */
    int32_t lat; /*< [degE7] Latitude (WGS84) */
    int32_t lon; /*< [degE7] Longitude (WGS84) */
    float alt_ellipsoid; /*< [m] Altitude (WGS84 elipsoid), preferred if available */
    float alt; /*< [m] Altitude (MSL - position-system specific value) use if no alt_ellipsoid available */
    float eph; /*< [m] Standard deviation of horizontal position error */
    float epv; /*< [m] Standard deviation of vertical position error */


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
        ss << "  id: " << +id << std::endl;
        ss << "  time_usec: " << time_usec << std::endl;
        ss << "  processing_time: " << processing_time << std::endl;
        ss << "  source: " << +source << std::endl;
        ss << "  flags: " << +flags << std::endl;
        ss << "  lat: " << lat << std::endl;
        ss << "  lon: " << lon << std::endl;
        ss << "  alt_ellipsoid: " << alt_ellipsoid << std::endl;
        ss << "  alt: " << alt << std::endl;
        ss << "  eph: " << eph << std::endl;
        ss << "  epv: " << epv << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << processing_time;               // offset: 8
        map << lat;                           // offset: 12
        map << lon;                           // offset: 16
        map << alt_ellipsoid;                 // offset: 20
        map << alt;                           // offset: 24
        map << eph;                           // offset: 28
        map << epv;                           // offset: 32
        map << target_system;                 // offset: 36
        map << target_component;              // offset: 37
        map << id;                            // offset: 38
        map << source;                        // offset: 39
        map << flags;                         // offset: 40
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> processing_time;               // offset: 8
        map >> lat;                           // offset: 12
        map >> lon;                           // offset: 16
        map >> alt_ellipsoid;                 // offset: 20
        map >> alt;                           // offset: 24
        map >> eph;                           // offset: 28
        map >> epv;                           // offset: 32
        map >> target_system;                 // offset: 36
        map >> target_component;              // offset: 37
        map >> id;                            // offset: 38
        map >> source;                        // offset: 39
        map >> flags;                         // offset: 40
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
