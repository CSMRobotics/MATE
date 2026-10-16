// MESSAGE FUEL_STATUS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief FUEL_STATUS message
 *
 * Fuel status.
        This message provides "generic" fuel level information for  in a GCS and for triggering failsafes in an autopilot.
        The fuel type and associated units for fields in this message are defined in the enum MAV_FUEL_TYPE.

        The reported `consumed_fuel` and `remaining_fuel` must only be supplied if measured: they must not be inferred from the `maximum_fuel` and the other value.
        A recipient can assume that if these fields are supplied they are accurate.
        If not provided, the recipient can infer `remaining_fuel` from `maximum_fuel` and `consumed_fuel` on the assumption that the fuel was initially at its maximum (this is what battery monitors assume).
        Note however that this is an assumption, and the UI should prompt the user appropriately (i.e. notify user that they should fill the tank before boot).

        This kind of information may also be sent in fuel-specific messages such as BATTERY_STATUS_V2.
        If both messages are sent for the same fuel system, the ids and corresponding information must match.

        This should be streamed (nominally at 0.1 Hz).
      
 */
struct FUEL_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 371;
    static constexpr size_t LENGTH = 26;
    static constexpr size_t MIN_LENGTH = 26;
    static constexpr uint8_t CRC_EXTRA = 10;
    static constexpr auto NAME = "FUEL_STATUS";


    uint8_t id; /*<  Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2. */
    float maximum_fuel; /*<  Capacity when full. Must be provided. */
    float consumed_fuel; /*<  Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided. */
    float remaining_fuel; /*<  Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided. */
    uint8_t percent_remaining; /*< [%] Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided. */
    float flow_rate; /*<  Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided. */
    float temperature; /*< [K] Fuel temperature. NaN: field not provided. */
    uint32_t fuel_type; /*<  Fuel type. Defines units for fuel capacity and consumption fields above. */


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
        ss << "  maximum_fuel: " << maximum_fuel << std::endl;
        ss << "  consumed_fuel: " << consumed_fuel << std::endl;
        ss << "  remaining_fuel: " << remaining_fuel << std::endl;
        ss << "  percent_remaining: " << +percent_remaining << std::endl;
        ss << "  flow_rate: " << flow_rate << std::endl;
        ss << "  temperature: " << temperature << std::endl;
        ss << "  fuel_type: " << fuel_type << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << maximum_fuel;                  // offset: 0
        map << consumed_fuel;                 // offset: 4
        map << remaining_fuel;                // offset: 8
        map << flow_rate;                     // offset: 12
        map << temperature;                   // offset: 16
        map << fuel_type;                     // offset: 20
        map << id;                            // offset: 24
        map << percent_remaining;             // offset: 25
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> maximum_fuel;                  // offset: 0
        map >> consumed_fuel;                 // offset: 4
        map >> remaining_fuel;                // offset: 8
        map >> flow_rate;                     // offset: 12
        map >> temperature;                   // offset: 16
        map >> fuel_type;                     // offset: 20
        map >> id;                            // offset: 24
        map >> percent_remaining;             // offset: 25
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
