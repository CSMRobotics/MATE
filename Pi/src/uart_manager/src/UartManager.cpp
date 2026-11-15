#include "uart_manager/UartManager.h"

#include <iostream>
#include <termios.h>
#include <cerrno>
#include <cstring>
#include <chrono>

UartManager::UartManager() : rclcpp::Node("uart_manager") {
    // Define and load params
    _load_config();

    // Open UART port
    RCLCPP_INFO(this->get_logger(), "Starting UART MAVLink Manager on serial port %s", _serial_port_path.c_str());
    _open_serial_port();

    // Start multithreading
    _run_threads.store(true);
    _micro_to_pi_thread = std::thread(
        &UartManager::micro_to_pi, 
        this, 
        _pub_channels, 
        _micro_pi_poll_rate, 
        _serial_fd
    );
    _pi_to_micro_thread = std::thread(
        &UartManager::pi_to_micro, 
        this, 
        _sub_channels_norm,
        _sub_channels_priority,
        _pi_micro_poll_rate,
        _serial_fd
    );
}

UartManager::~UartManager() {
    shutdown();
}

void UartManager::shutdown() {
    RCLCPP_INFO(this->get_logger(), "Shutting down node");
    // Join threads
    _run_threads.store(false);
    if (_pi_to_micro_thread.joinable()) {
        _pi_to_micro_thread.join();
    }
    if (_micro_to_pi_thread.joinable()) {
        _micro_to_pi_thread.join();
    }

    // Close serial ports
    if (_serial_fd >= 0) {
        close(_serial_fd);
    }
}


void UartManager::_load_config() {
    // Declare parameters
    this->declare_parameter<std::string>("serial_port", "/dev/ttyAMA10");
    this->declare_parameter<int64_t>("tx_polling_rate", 100);
    this->declare_parameter<int64_t>("rx_polling_rate", 100);

    this->declare_parameter<std::vector<std::string>>("tx_channel_names", std::vector<std::string>{});
    this->declare_parameter<std::vector<int64_t>>("tx_channel_priorities", std::vector<int64_t>{});

    this->declare_parameter<std::vector<std::string>>("rx_channel_names", std::vector<std::string>{});
    this->declare_parameter<std::vector<int64_t>>("rx_message_ids", std::vector<int64_t>{});

    // Load easy parameters
    this->get_parameter("serial_port", _serial_port_path);
    
    _pi_micro_poll_rate = this->get_parameter("tx_polling_rate").as_int();
    _micro_pi_poll_rate = this->get_parameter("rx_polling_rate").as_int();

    // Load micro->Pi parameters
    std::vector<std::string> rx_channel_names = this->get_parameter("rx_channel_names").as_string_array();
    std::vector<int64_t> message_ids = this->get_parameter("rx_message_ids").as_integer_array();

    if (rx_channel_names.size() != message_ids.size()) {
        RCLCPP_ERROR(this->get_logger(), "Mismatched sizes in RX channel names and message ids configuration arrays");
        this->shutdown();
        rclcpp::shutdown();
        return;
    }

    for (size_t i = 0; i < rx_channel_names.size(); ++i) {
        std::string name = rx_channel_names[i];
        int64_t message_id = message_ids[i];
        try {
            this->_pub_channels[message_id] = std::make_shared<MavlinkPubSub>(true, name);
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded RX channel with name: '%s', message_id: %ld", 
                name.c_str(), 
                message_id
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            this->shutdown();
            rclcpp::shutdown();
            return;
        }
    }

    // Load Pi->micro parameters
    std::vector<std::string> tx_channel_names = this->get_parameter("tx_channel_names").as_string_array();
    std::vector<int64_t> priorities = this->get_parameter("tx_channel_priorities").as_integer_array();

    if (tx_channel_names.size() != priorities.size()) {
        RCLCPP_ERROR(this->get_logger(), "Mismatched sizes in TX channel names and priorities configuration arrays");
        this->shutdown();
        rclcpp::shutdown();
        return;
    }

    for (size_t i = 0; i < tx_channel_names.size(); ++i) {
        std::string name = tx_channel_names[i];
        int64_t priority = priorities[i];
        try {
            if (priority > 0) {
                this->_sub_channels_priority.push_back(std::make_shared<MavlinkPubSub>(false, name));
            } else {
                this->_sub_channels_norm.push_back(std::make_shared<MavlinkPubSub>(false, name));
            }
            RCLCPP_INFO(
                this->get_logger(),
                "Loaded TX channel with name: '%s', priority: %ld", 
                name.c_str(), 
                priority
            );
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load config: %s", e.what());
            this->shutdown();
            rclcpp::shutdown();
            return;
        }
    }
}


void UartManager::_open_serial_port() {
    _serial_fd = open(_serial_port_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (_serial_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", _serial_port_path.c_str());
        this->shutdown();
        rclcpp::shutdown();
    }
    _configure_serial_port();
}

void UartManager::_configure_serial_port() {
    struct termios tty;
    if (tcgetattr(_serial_fd, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error getting termios attributes %s", std::strerror(errno));
        this->shutdown();
        rclcpp::shutdown();
    }

    // Set Baud Rate to 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Set 8N1 raw mode
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);

    tcflush(_serial_fd, TCIFLUSH);
    if (tcsetattr(_serial_fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error setting termios attributes %s", std::strerror(errno));
        this->shutdown();
        rclcpp::shutdown();
    }
}


//*****************************************************************
// Multithreaded reader and writers
void UartManager::micro_to_pi(
    const std::unordered_map<int, std::shared_ptr<MavlinkPubSub>>& pub_channels,
    const int polling_rate,
    const int serial_fd
) {
    // Setup polling loop
    if (polling_rate <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid polling rate: %d", polling_rate);
        return;
    }
    const auto update_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / polling_rate)
    );
    auto desired_period = std::chrono::duration<double, std::milli>(std::chrono::duration<double>(1.0 / polling_rate));
    auto next_tick = std::chrono::steady_clock::now();
    auto prev_tick = next_tick;


    // Ze loop
    while (_run_threads.load()) {
        // Read a message from serial port


        // Check if polling rate is met
        auto actual_period = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - prev_tick);
        if (actual_period > desired_period + (0.2 * desired_period)) {
            RCLCPP_WARN(this->get_logger(), "UART RX update period exceeded expectation: Actual: %.3f, Desired: %.3f", actual_period.count(), desired_period.count());
        }
        prev_tick = std::chrono::steady_clock::now();

        // Delay until next polling cycle
        next_tick += update_period;
        std::this_thread::sleep_until(next_tick);
    }
}

void UartManager::pi_to_micro(
    const std::vector<std::shared_ptr<MavlinkPubSub>>& sub_channels_norm,
    const std::vector<std::shared_ptr<MavlinkPubSub>>& sub_channels_priority,
    const int polling_rate,
    const int serial_fd
) {
    // Setup polling loop
    if (polling_rate <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid polling rate: %d", polling_rate);
        return;
    }
    const auto update_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / polling_rate)
    );
    const auto desired_period = std::chrono::duration<double, std::milli>(update_period);
    auto next_tick = std::chrono::steady_clock::now();
    auto prev_tick = next_tick;

    // Ze loop
    while (_run_threads.load()) {
        // Round robin priority channels first
        for (size_t i = 0; i < sub_channels_priority.size(); i++) {
            // Read in message
            const std::shared_ptr<MavlinkPubSub>& sub = sub_channels_priority[i];
            mavlink_message_t msg{};
            mavlink_status_t status{};
            if (sub->read(msg, status) != 1) {
                continue;
            }

            // Send it
            uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
            ssize_t bytes_written = write(serial_fd, tx_buffer, len);

            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send over UART!!!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent Priority MAVLink Message!");
            }
        }

        // Round robin normal channels
        for (size_t i = 0; i < _sub_channels_norm.size(); i++) {
            // Read in message
            const std::shared_ptr<MavlinkPubSub>& sub = _sub_channels_norm[i];
            mavlink_message_t msg{};
            mavlink_status_t status{};
            if (sub->read(msg, status) != 1) {
                continue;
            }

            // Send it
            uint8_t tx_buffer[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(tx_buffer, &msg);
            ssize_t bytes_written = write(serial_fd, tx_buffer, len);

            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to send over UART!!!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Sent MAVLink Message!");
            }
        }

        // Delay until next polling cycle
        next_tick += update_period;

        // Check if polling rate is met
        auto actual_period = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - prev_tick);
        if (actual_period > desired_period + (0.2 * desired_period)) {
            RCLCPP_WARN(this->get_logger(), "UART TX update period exceeded expectation: Actual: %.3f, Desired: %.3f", actual_period.count(), desired_period.count());
        }
        prev_tick = std::chrono::steady_clock::now();

        std::this_thread::sleep_until(next_tick);
    }
}

