// MESSAGE CAMERA_THERMAL_RANGE support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CAMERA_THERMAL_RANGE message
 *
 * Camera absolute thermal range. This can be streamed when the associated VIDEO_STREAM_STATUS `flag` field bit VIDEO_STREAM_STATUS_FLAGS_THERMAL_RANGE_ENABLED is set, but a GCS may choose to only request it for the current active stream. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message interval (param3 indicates the stream id of the current camera, or 0 for all streams, param4 indicates the target camera_device_id for autopilot-attached cameras or 0 for MAVLink cameras).
 */
struct CAMERA_THERMAL_RANGE : mavlink::Message {
    static constexpr msgid_t MSG_ID = 277;
    static constexpr size_t LENGTH = 30;
    static constexpr size_t MIN_LENGTH = 30;
    static constexpr uint8_t CRC_EXTRA = 62;
    static constexpr auto NAME = "CAMERA_THERMAL_RANGE";


    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot). */
    uint8_t stream_id; /*<  Video Stream ID (1 for first, 2 for second, etc.) */
    uint8_t camera_device_id; /*<  Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id). */
    float max; /*< [degC] Temperature max. */
    float max_point_x; /*<  Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown. */
    float max_point_y; /*<  Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown. */
    float min; /*< [degC] Temperature min. */
    float min_point_x; /*<  Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown. */
    float min_point_y; /*<  Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown. */


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
        ss << "  stream_id: " << +stream_id << std::endl;
        ss << "  camera_device_id: " << +camera_device_id << std::endl;
        ss << "  max: " << max << std::endl;
        ss << "  max_point_x: " << max_point_x << std::endl;
        ss << "  max_point_y: " << max_point_y << std::endl;
        ss << "  min: " << min << std::endl;
        ss << "  min_point_x: " << min_point_x << std::endl;
        ss << "  min_point_y: " << min_point_y << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_boot_ms;                  // offset: 0
        map << max;                           // offset: 4
        map << max_point_x;                   // offset: 8
        map << max_point_y;                   // offset: 12
        map << min;                           // offset: 16
        map << min_point_x;                   // offset: 20
        map << min_point_y;                   // offset: 24
        map << stream_id;                     // offset: 28
        map << camera_device_id;              // offset: 29
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_boot_ms;                  // offset: 0
        map >> max;                           // offset: 4
        map >> max_point_x;                   // offset: 8
        map >> max_point_y;                   // offset: 12
        map >> min;                           // offset: 16
        map >> min_point_x;                   // offset: 20
        map >> min_point_y;                   // offset: 24
        map >> stream_id;                     // offset: 28
        map >> camera_device_id;              // offset: 29
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
