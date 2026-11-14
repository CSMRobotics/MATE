// MESSAGE SYS_STATUS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief SYS_STATUS message
 *
 * Sensor and subsystem status information. Provides a compact representation of sensor/subsystem status and a few other basic statistics.
 */
struct SYS_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 1;
    static constexpr size_t LENGTH = 43;
    static constexpr size_t MIN_LENGTH = 31;
    static constexpr uint8_t CRC_EXTRA = 124;
    static constexpr auto NAME = "SYS_STATUS";


    uint32_t onboard_control_sensors_present; /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. */
    uint32_t onboard_control_sensors_enabled; /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. */
    uint32_t onboard_control_sensors_health; /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy. */
    uint16_t load; /*< [d%] Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000 */
    uint16_t voltage_battery; /*< [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot. Value is ambiguous on multi-battery systems. BATTERY_STATUS is a recommended alternative. */
    int16_t current_battery; /*< [cA] Battery current, -1: Current not sent by autopilot. Value may overflow/rollover for very high currents (> 327.67A). Value is ambiguous on multi-battery systems. BATTERY_STATUS is a recommended alternative. */
    int8_t battery_remaining; /*< [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot. Value is ambiguous on multi-battery systems. BATTERY_STATUS is a recommended alternative. */
    uint16_t drop_rate_comm; /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) */
    uint16_t errors_comm; /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV) */
    uint16_t errors_count1; /*<  Autopilot-specific errors */
    uint16_t errors_count2; /*<  Autopilot-specific errors */
    uint16_t errors_count3; /*<  Autopilot-specific errors */
    uint16_t errors_count4; /*<  Autopilot-specific errors */
    uint32_t onboard_control_sensors_present_extended; /*<  Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. */
    uint32_t onboard_control_sensors_enabled_extended; /*<  Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. */
    uint32_t onboard_control_sensors_health_extended; /*<  Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy. */


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
        ss << "  onboard_control_sensors_present: " << onboard_control_sensors_present << std::endl;
        ss << "  onboard_control_sensors_enabled: " << onboard_control_sensors_enabled << std::endl;
        ss << "  onboard_control_sensors_health: " << onboard_control_sensors_health << std::endl;
        ss << "  load: " << load << std::endl;
        ss << "  voltage_battery: " << voltage_battery << std::endl;
        ss << "  current_battery: " << current_battery << std::endl;
        ss << "  battery_remaining: " << +battery_remaining << std::endl;
        ss << "  drop_rate_comm: " << drop_rate_comm << std::endl;
        ss << "  errors_comm: " << errors_comm << std::endl;
        ss << "  errors_count1: " << errors_count1 << std::endl;
        ss << "  errors_count2: " << errors_count2 << std::endl;
        ss << "  errors_count3: " << errors_count3 << std::endl;
        ss << "  errors_count4: " << errors_count4 << std::endl;
        ss << "  onboard_control_sensors_present_extended: " << onboard_control_sensors_present_extended << std::endl;
        ss << "  onboard_control_sensors_enabled_extended: " << onboard_control_sensors_enabled_extended << std::endl;
        ss << "  onboard_control_sensors_health_extended: " << onboard_control_sensors_health_extended << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << onboard_control_sensors_present; // offset: 0
        map << onboard_control_sensors_enabled; // offset: 4
        map << onboard_control_sensors_health; // offset: 8
        map << load;                          // offset: 12
        map << voltage_battery;               // offset: 14
        map << current_battery;               // offset: 16
        map << drop_rate_comm;                // offset: 18
        map << errors_comm;                   // offset: 20
        map << errors_count1;                 // offset: 22
        map << errors_count2;                 // offset: 24
        map << errors_count3;                 // offset: 26
        map << errors_count4;                 // offset: 28
        map << battery_remaining;             // offset: 30
        map << onboard_control_sensors_present_extended; // offset: 31
        map << onboard_control_sensors_enabled_extended; // offset: 35
        map << onboard_control_sensors_health_extended; // offset: 39
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> onboard_control_sensors_present; // offset: 0
        map >> onboard_control_sensors_enabled; // offset: 4
        map >> onboard_control_sensors_health; // offset: 8
        map >> load;                          // offset: 12
        map >> voltage_battery;               // offset: 14
        map >> current_battery;               // offset: 16
        map >> drop_rate_comm;                // offset: 18
        map >> errors_comm;                   // offset: 20
        map >> errors_count1;                 // offset: 22
        map >> errors_count2;                 // offset: 24
        map >> errors_count3;                 // offset: 26
        map >> errors_count4;                 // offset: 28
        map >> battery_remaining;             // offset: 30
        map >> onboard_control_sensors_present_extended; // offset: 31
        map >> onboard_control_sensors_enabled_extended; // offset: 35
        map >> onboard_control_sensors_health_extended; // offset: 39
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
