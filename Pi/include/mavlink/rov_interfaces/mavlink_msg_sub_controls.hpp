// MESSAGE SUB_CONTROLS support class

#pragma once

namespace mavlink {
namespace rov_interfaces {
namespace msg {

/**
 * @brief SUB_CONTROLS message
 *
 * 
                Submarine control inputs. All control values are normalized and unitless.
            
 */
struct SUB_CONTROLS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 200;
    static constexpr size_t LENGTH = 26;
    static constexpr size_t MIN_LENGTH = 26;
    static constexpr uint8_t CRC_EXTRA = 17;
    static constexpr auto NAME = "SUB_CONTROLS";


    float velocity_x; /*<  
                Forward velocity (+Forward)
             */
    float velocity_y; /*<  
                Lateral velocity (+Right)
             */
    float velocity_z; /*<  
                Vertical velocity (+Down)
             */
    float omega_x; /*<  
                Roll rate (+Clockwise looking down x axis, from back to front)
             */
    float omega_y; /*<  
                Pitch rate (+Pull up)
             */
    float omega_z; /*<  
                Yaw rate (+Turn right)
             */
    uint8_t mode_switch; /*<  
                Mode switch (0 = disabled, 1 = enabled)
             */
    uint8_t estop; /*<  
                Emergency stop (0 = inactive, 1 = active)
             */


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
        ss << "  velocity_x: " << velocity_x << std::endl;
        ss << "  velocity_y: " << velocity_y << std::endl;
        ss << "  velocity_z: " << velocity_z << std::endl;
        ss << "  omega_x: " << omega_x << std::endl;
        ss << "  omega_y: " << omega_y << std::endl;
        ss << "  omega_z: " << omega_z << std::endl;
        ss << "  mode_switch: " << +mode_switch << std::endl;
        ss << "  estop: " << +estop << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << velocity_x;                    // offset: 0
        map << velocity_y;                    // offset: 4
        map << velocity_z;                    // offset: 8
        map << omega_x;                       // offset: 12
        map << omega_y;                       // offset: 16
        map << omega_z;                       // offset: 20
        map << mode_switch;                   // offset: 24
        map << estop;                         // offset: 25
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> velocity_x;                    // offset: 0
        map >> velocity_y;                    // offset: 4
        map >> velocity_z;                    // offset: 8
        map >> omega_x;                       // offset: 12
        map >> omega_y;                       // offset: 16
        map >> omega_z;                       // offset: 20
        map >> mode_switch;                   // offset: 24
        map >> estop;                         // offset: 25
    }
};

} // namespace msg
} // namespace rov_interfaces
} // namespace mavlink
