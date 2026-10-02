// MESSAGE RADIO_RC_CHANNELS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief RADIO_RC_CHANNELS message
 *
 * RC channel outputs from a MAVLink RC receiver for input to a flight controller or other components (allows an RC receiver to connect via MAVLink instead of some other protocol such as PPM-Sum or S.BUS).
        Note that this is not intended to be an over-the-air format, and does not replace RC_CHANNELS and similar messages reported by the flight controller.
        The target_system field should normally be set to the system id of the system to control, typically the flight controller.
        The target_component field can normally be set to 0, so that all components of the system can receive the message.
        The channels array field can publish up to 32 channels; the number of channel items used in the array is specified in the count field.
        The time_last_update_ms field contains the timestamp of the last received valid channels data in the receiver's time domain.
        The count field indicates the first index of the channel array that is not used for channel data (this and later indexes are zero-filled).
        The RADIO_RC_CHANNELS_FLAGS_OUTDATED flag is set by the receiver if the channels data is not up-to-date (for example, if new data from the transmitter could not be validated so the last valid data is resent).
        The RADIO_RC_CHANNELS_FLAGS_FAILSAFE failsafe flag is set by the receiver if the receiver's failsafe condition is met (implementation dependent, e.g., connection to the RC radio is lost).
        In this case time_last_update_ms still contains the timestamp of the last valid channels data, but the content of the channels data is not defined by the protocol (it is up to the implementation of the receiver).
        For instance, the channels data could contain failsafe values configured in the receiver; the default is to carry the last valid data.
        Note: The RC channels fields are extensions to ensure that they are located at the end of the serialized payload and subject to MAVLink's trailing-zero trimming.
      
 */
struct RADIO_RC_CHANNELS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 420;
    static constexpr size_t LENGTH = 73;
    static constexpr size_t MIN_LENGTH = 9;
    static constexpr uint8_t CRC_EXTRA = 20;
    static constexpr auto NAME = "RADIO_RC_CHANNELS";


    uint8_t target_system; /*<  System ID (ID of target system, normally flight controller). */
    uint8_t target_component; /*<  Component ID (normally 0 for broadcast). */
    uint32_t time_last_update_ms; /*< [ms] Time when the data in the channels field were last updated (time since boot in the receiver's time domain). */
    uint16_t flags; /*<  Radio RC channels status flags. */
    uint8_t count; /*<  Total number of RC channels being received. This can be larger than 32, indicating that more channels are available but not given in this message. */
    std::array<int16_t, 32> channels; /*<  RC channels.
        Channel values are in centered 13 bit format. Range is -4096 to 4096, center is 0. Conversion to PWM is x * 5/32 + 1500.
        Channels with indexes equal or above count should be set to 0, to benefit from MAVLink's trailing-zero trimming. */


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
        ss << "  time_last_update_ms: " << time_last_update_ms << std::endl;
        ss << "  flags: " << flags << std::endl;
        ss << "  count: " << +count << std::endl;
        ss << "  channels: [" << to_string(channels) << "]" << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_last_update_ms;           // offset: 0
        map << flags;                         // offset: 4
        map << target_system;                 // offset: 6
        map << target_component;              // offset: 7
        map << count;                         // offset: 8
        map << channels;                      // offset: 9
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_last_update_ms;           // offset: 0
        map >> flags;                         // offset: 4
        map >> target_system;                 // offset: 6
        map >> target_component;              // offset: 7
        map >> count;                         // offset: 8
        map >> channels;                      // offset: 9
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
