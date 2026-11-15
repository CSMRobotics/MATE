#include "MavlinkPubSub.hpp"
#include "mavlink/rov_interfaces/mavlink.h"

#include <thread>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class UartManager : public rclcpp::Node {
public:
    /**
     * @brief Constructor
     */
    UartManager();

    /**
     * @brief Shutdown and cleanup the node
     */
    ~UartManager();

    /**
     * @brief Shutdown and cleanup the node
     */
    void shutdown();

private:
    // Startup helpers
    void _load_config();
    void _open_serial_port();
    void _configure_serial_port();

    //*****************************************************************
    // Shared information
    int _serial_fd = -1;
    std::string _serial_port_path = "";

    std::unordered_map<int, std::shared_ptr<MavlinkPubSub>> _pub_channels = {};  // MavlinkMsgID: PubSubChannel, for quick lookup upon msg receive
    std::vector<std::shared_ptr<MavlinkPubSub>> _sub_channels_norm = {};
    std::vector<std::shared_ptr<MavlinkPubSub>> _sub_channels_priority = {};   // Higher priority messages when scheuduling transmits

    int _micro_pi_poll_rate;    // Hz
    int _pi_micro_poll_rate;

    //*****************************************************************
    // Multithreaded reader and writers
    std::thread _micro_to_pi_thread;
    std::thread _pi_to_micro_thread;
    std::atomic<bool> _run_threads{true};   // If false, threads should exit

    /**
     * @brief Receives data from microcontroller and publishes to mav_pubs processes
     * 
     * @param pub_channels: Pointers to MavlinkPubSub publishers
     * @param polling_rate: Polling rate of serial port
     * @param serial_fd: File descriptor of serial port
     */
    void micro_to_pi(
        const std::unordered_map<int, std::shared_ptr<MavlinkPubSub>>& pub_channels,
        const int polling_rate,
        const int serial_fd
    );

    /**
     * @brief Transmits data from other processes to the microcontroller
     * 
     * @param sub_channels_norm: MavlinkPubSub channels to transmit to microcontroller
     * @param sub_channels_priority: Higher priority channels to transmit first
     * @param polling_rate: Polling rate of each channel
     * @param serial_fd: File descriptor of serial port
     */
    void pi_to_micro(
        const std::vector<std::shared_ptr<MavlinkPubSub>>& sub_channels_norm, 
        const std::vector<std::shared_ptr<MavlinkPubSub>>& sub_channels_priority,
        const int polling_rate,
        const int serial_fd
    );
};