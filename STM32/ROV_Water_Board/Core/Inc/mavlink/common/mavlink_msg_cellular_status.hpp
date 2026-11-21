// MESSAGE CELLULAR_STATUS support class

#pragma once

namespace mavlink {
namespace common {
namespace msg {

/**
 * @brief CELLULAR_STATUS message
 *
 * Cellular network status as reported by a particular modem.

        This is primarily intended for logging, but a GCS may choose to display link_tx_rate and link_rx_rate.

        Note that a value of 0 in the id field indicates that the sender does not support reporting of multiple modems.
        Message data should be from a single modem, but that is not guaranteed.
      
 */
struct CELLULAR_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 334;
    static constexpr size_t LENGTH = 53;
    static constexpr size_t MIN_LENGTH = 10;
    static constexpr uint8_t CRC_EXTRA = 72;
    static constexpr auto NAME = "CELLULAR_STATUS";


    uint8_t status; /*<  Cellular modem status */
    uint8_t failure_reason; /*<  Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED */
    uint8_t type; /*<  Cellular network radio type: gsm, cdma, lte... */
    uint8_t quality; /*<  Signal quality in percent. If unknown, set to UINT8_MAX */
    uint16_t mcc; /*<  Mobile country code. If unknown, set to UINT16_MAX */
    uint16_t mnc; /*<  Mobile network code. If unknown, set to UINT16_MAX */
    uint16_t lac; /*<  Location area code. If unknown, set to 0 */
    uint8_t id; /*<  Cellular modem instance number. Indexed from 1. */
    uint32_t link_tx_rate; /*< [KiB/s] Download rate. */
    uint32_t link_rx_rate; /*< [KiB/s] Upload rate. */
    std::array<char, 9> cell_tower_id; /*<  ID of the currently connected cell tower. This must be NULL terminated if the length is less than 9 human-readable chars, and without the null termination (NULL) byte if the length is exactly 9 chars. */
    uint8_t band_number; /*<  LTE frequency band number. */
    float band_frequency; /*< [MHz] LTE radio frequency. */
    uint32_t channel_number; /*<  The channel number (CN). Absolute radio-frequency (ARFCN) / E-UTRA (EARFCN) / UTRA (UARFCN) / New radio (NR_CH). */
    float rx_level; /*< [dBm] On 3G is Received Signal Code Power (RSCP). On LTE is Reference Signal Received Power (RSRP). On 5G is New Radio Reference Signal Received Power (NR_RSRP). */
    float tx_level; /*< [dBm] Transmitter (modem) signal absolute power level. */
    float rx_quality; /*< [dBm] On 3G is Receiver Quality (RxQual). On LTE is Reference Signal Received Quality (RSRQ). On 5G is New Radio Reference Signal Received Quality (NR_RSRQ). */
    float sinr; /*< [dB] Signal to interference plus noise ratio (SINR). */


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
        ss << "  status: " << +status << std::endl;
        ss << "  failure_reason: " << +failure_reason << std::endl;
        ss << "  type: " << +type << std::endl;
        ss << "  quality: " << +quality << std::endl;
        ss << "  mcc: " << mcc << std::endl;
        ss << "  mnc: " << mnc << std::endl;
        ss << "  lac: " << lac << std::endl;
        ss << "  id: " << +id << std::endl;
        ss << "  link_tx_rate: " << link_tx_rate << std::endl;
        ss << "  link_rx_rate: " << link_rx_rate << std::endl;
        ss << "  cell_tower_id: \"" << to_string(cell_tower_id) << "\"" << std::endl;
        ss << "  band_number: " << +band_number << std::endl;
        ss << "  band_frequency: " << band_frequency << std::endl;
        ss << "  channel_number: " << channel_number << std::endl;
        ss << "  rx_level: " << rx_level << std::endl;
        ss << "  tx_level: " << tx_level << std::endl;
        ss << "  rx_quality: " << rx_quality << std::endl;
        ss << "  sinr: " << sinr << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << mcc;                           // offset: 0
        map << mnc;                           // offset: 2
        map << lac;                           // offset: 4
        map << status;                        // offset: 6
        map << failure_reason;                // offset: 7
        map << type;                          // offset: 8
        map << quality;                       // offset: 9
        map << id;                            // offset: 10
        map << link_tx_rate;                  // offset: 11
        map << link_rx_rate;                  // offset: 15
        map << cell_tower_id;                 // offset: 19
        map << band_number;                   // offset: 28
        map << band_frequency;                // offset: 29
        map << channel_number;                // offset: 33
        map << rx_level;                      // offset: 37
        map << tx_level;                      // offset: 41
        map << rx_quality;                    // offset: 45
        map << sinr;                          // offset: 49
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> mcc;                           // offset: 0
        map >> mnc;                           // offset: 2
        map >> lac;                           // offset: 4
        map >> status;                        // offset: 6
        map >> failure_reason;                // offset: 7
        map >> type;                          // offset: 8
        map >> quality;                       // offset: 9
        map >> id;                            // offset: 10
        map >> link_tx_rate;                  // offset: 11
        map >> link_rx_rate;                  // offset: 15
        map >> cell_tower_id;                 // offset: 19
        map >> band_number;                   // offset: 28
        map >> band_frequency;                // offset: 29
        map >> channel_number;                // offset: 33
        map >> rx_level;                      // offset: 37
        map >> tx_level;                      // offset: 41
        map >> rx_quality;                    // offset: 45
        map >> sinr;                          // offset: 49
    }
};

} // namespace msg
} // namespace common
} // namespace mavlink
