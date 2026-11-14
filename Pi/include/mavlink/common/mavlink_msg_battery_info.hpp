// MESSAGE BATTERY_INFO support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief BATTERY_INFO message
 *
 * 
        Battery information that is static, or requires infrequent update.
        This message should requested using MAV_CMD_REQUEST_MESSAGE and/or streamed at very low rate.
        BATTERY_STATUS_V2 is used for higher-rate battery status information.
      
 */
struct BATTERY_INFO : mavlink::Message {
    static constexpr msgid_t MSG_ID = 372;
    static constexpr size_t LENGTH = 140;
    static constexpr size_t MIN_LENGTH = 140;
    static constexpr uint8_t CRC_EXTRA = 26;
    static constexpr auto NAME = "BATTERY_INFO";


    uint8_t id; /*<  Battery ID */
    uint8_t battery_function; /*<  Function of the battery. */
    uint8_t type; /*<  Type (chemistry) of the battery. */
    uint8_t state_of_health; /*< [%] State of Health (SOH) estimate. Typically 100% at the time of manufacture and will decrease over time and use. -1: field not provided. */
    uint8_t cells_in_series; /*<  Number of battery cells in series. 0: field not provided. */
    uint16_t cycle_count; /*<  Lifetime count of the number of charge/discharge cycles (https://en.wikipedia.org/wiki/Charge_cycle). UINT16_MAX: field not provided. */
    uint16_t weight; /*< [g] Battery weight. 0: field not provided. */
    float discharge_minimum_voltage; /*< [V] Minimum per-cell voltage when discharging. 0: field not provided. */
    float charging_minimum_voltage; /*< [V] Minimum per-cell voltage when charging. 0: field not provided. */
    float resting_minimum_voltage; /*< [V] Minimum per-cell voltage when resting. 0: field not provided. */
    float charging_maximum_voltage; /*< [V] Maximum per-cell voltage when charged. 0: field not provided. */
    float charging_maximum_current; /*< [A] Maximum pack continuous charge current. 0: field not provided. */
    float nominal_voltage; /*< [V] Battery nominal voltage. Used for conversion between Wh and Ah. 0: field not provided. */
    float discharge_maximum_current; /*< [A] Maximum pack discharge current. 0: field not provided. */
    float discharge_maximum_burst_current; /*< [A] Maximum pack discharge burst current. 0: field not provided. */
    float design_capacity; /*< [Ah] Fully charged design capacity. 0: field not provided. */
    float full_charge_capacity; /*< [Ah] Predicted battery capacity when fully charged (accounting for battery degradation). NAN: field not provided. */
    std::array<char, 9> manufacture_date; /*<  Manufacture date (DDMMYYYY) in ASCII characters, 0 terminated. All 0: field not provided. */
    std::array<char, 32> serial_number; /*<  Serial number in ASCII characters, 0 terminated. All 0: field not provided. */
    std::array<char, 50> name; /*<  Battery device name. Formatted as manufacturer name then product name, separated with an underscore (in ASCII characters), 0 terminated. All 0: field not provided. */


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
        ss << "  battery_function: " << +battery_function << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  state_of_health: " << +state_of_health << std::endl;
        ss << "  cells_in_series: " << +cells_in_series << std::endl;
        ss << "  cycle_count: " << cycle_count << std::endl;
        ss << "  weight: " << weight << std::endl;
        ss << "  discharge_minimum_voltage: " << discharge_minimum_voltage << std::endl;
        ss << "  charging_minimum_voltage: " << charging_minimum_voltage << std::endl;
        ss << "  resting_minimum_voltage: " << resting_minimum_voltage << std::endl;
        ss << "  charging_maximum_voltage: " << charging_maximum_voltage << std::endl;
        ss << "  charging_maximum_current: " << charging_maximum_current << std::endl;
        ss << "  nominal_voltage: " << nominal_voltage << std::endl;
        ss << "  discharge_maximum_current: " << discharge_maximum_current << std::endl;
        ss << "  discharge_maximum_burst_current: " << discharge_maximum_burst_current << std::endl;
        ss << "  design_capacity: " << design_capacity << std::endl;
        ss << "  full_charge_capacity: " << full_charge_capacity << std::endl;
        ss << "  manufacture_date: \"" << to_string(manufacture_date) << "\"" << std::endl;
        ss << "  serial_number: \"" << to_string(serial_number) << "\"" << std::endl;
        ss << "  name: \"" << to_string(name) << "\"" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << discharge_minimum_voltage;     // offset: 0
        map << charging_minimum_voltage;      // offset: 4
        map << resting_minimum_voltage;       // offset: 8
        map << charging_maximum_voltage;      // offset: 12
        map << charging_maximum_current;      // offset: 16
        map << nominal_voltage;               // offset: 20
        map << discharge_maximum_current;     // offset: 24
        map << discharge_maximum_burst_current; // offset: 28
        map << design_capacity;               // offset: 32
        map << full_charge_capacity;          // offset: 36
        map << cycle_count;                   // offset: 40
        map << weight;                        // offset: 42
        map << id;                            // offset: 44
        map << battery_function;              // offset: 45
        map << type;                          // offset: 46
        map << state_of_health;               // offset: 47
        map << cells_in_series;               // offset: 48
        map << manufacture_date;              // offset: 49
        map << serial_number;                 // offset: 58
        map << name;                          // offset: 90
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> discharge_minimum_voltage;     // offset: 0
        map >> charging_minimum_voltage;      // offset: 4
        map >> resting_minimum_voltage;       // offset: 8
        map >> charging_maximum_voltage;      // offset: 12
        map >> charging_maximum_current;      // offset: 16
        map >> nominal_voltage;               // offset: 20
        map >> discharge_maximum_current;     // offset: 24
        map >> discharge_maximum_burst_current; // offset: 28
        map >> design_capacity;               // offset: 32
        map >> full_charge_capacity;          // offset: 36
        map >> cycle_count;                   // offset: 40
        map >> weight;                        // offset: 42
        map >> id;                            // offset: 44
        map >> battery_function;              // offset: 45
        map >> type;                          // offset: 46
        map >> state_of_health;               // offset: 47
        map >> cells_in_series;               // offset: 48
        map >> manufacture_date;              // offset: 49
        map >> serial_number;                 // offset: 58
        map >> name;                          // offset: 90
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
