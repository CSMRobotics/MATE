// MESSAGE FIGURE_EIGHT_EXECUTION_STATUS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief FIGURE_EIGHT_EXECUTION_STATUS message
 *
 * 
        Vehicle status report that is sent out while figure eight execution is in progress (see MAV_CMD_DO_FIGURE_EIGHT).
        This may typically send at low rates: of the order of 2Hz.
      
 */
struct FIGURE_EIGHT_EXECUTION_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 361;
    static constexpr size_t LENGTH = 33;
    static constexpr size_t MIN_LENGTH = 33;
    static constexpr uint8_t CRC_EXTRA = 93;
    static constexpr auto NAME = "FIGURE_EIGHT_EXECUTION_STATUS";


    uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. */
    float major_radius; /*< [m] Major axis radius of the figure eight. Positive: orbit the north circle clockwise. Negative: orbit the north circle counter-clockwise. */
    float minor_radius; /*< [m] Minor axis radius of the figure eight. Defines the radius of two circles that make up the figure. */
    float orientation; /*< [rad] Orientation of the figure eight major axis with respect to true north in [-pi,pi). */
    uint8_t frame; /*<  The coordinate system of the fields: x, y, z. */
    int32_t x; /*<  X coordinate of center point. Coordinate system depends on frame field. */
    int32_t y; /*<  Y coordinate of center point. Coordinate system depends on frame field. */
    float z; /*< [m] Altitude of center point. Coordinate system depends on frame field. */


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
        ss << "  time_usec: " << time_usec << std::endl;
        ss << "  major_radius: " << major_radius << std::endl;
        ss << "  minor_radius: " << minor_radius << std::endl;
        ss << "  orientation: " << orientation << std::endl;
        ss << "  frame: " << +frame << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << major_radius;                  // offset: 8
        map << minor_radius;                  // offset: 12
        map << orientation;                   // offset: 16
        map << x;                             // offset: 20
        map << y;                             // offset: 24
        map << z;                             // offset: 28
        map << frame;                         // offset: 32
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> major_radius;                  // offset: 8
        map >> minor_radius;                  // offset: 12
        map >> orientation;                   // offset: 16
        map >> x;                             // offset: 20
        map >> y;                             // offset: 24
        map >> z;                             // offset: 28
        map >> frame;                         // offset: 32
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
