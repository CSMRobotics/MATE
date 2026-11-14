#include "MavlinkPubSub.hpp"
#include "mavlink/rov_interfaces/mavlink.h"

#include <thread>
#include <string>
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
    std::string _serial_port;
    std::vector<std::shared_ptr<MavlinkPubSub>> _mav_pubs;
    std::vector<std::shared_ptr<MavlinkPubSub>> _mav_subs;

    //*****************************************************************
    // Multithreaded reader and writers
    std::thread _micro_to_pi_thread;
    std::thread _pi_to_micro_thread;
    std::atomic<bool> _run_threads{true};   // If false, threads should exit

    /**
     * @brief Receives data from microcontroller and publishes to mav_pubs processes
     * 
     * @param mav_pubs: Pointers to MavlinkPubSub publishers
     */
    void micro_to_pi(std::vector<std::shared_ptr<MavlinkPubSub>> mav_pubs);

    /**
     * @brief Transmits data from other processes to the microcontroller
     * 
     * @param mav_subs: Pointers to MavlinkPubSub subscribers
     */
    void pi_to_micro(std::vector<std::shared_ptr<MavlinkPubSub>> mav_subs);
    
};