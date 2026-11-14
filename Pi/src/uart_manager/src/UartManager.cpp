#include "uart_manager/UartManager.h"

UartManager::UartManager() : rclcpp::Node("uart_manager") {
    // Define and load params
    _mav_pubs = {
        std::make_shared<MavlinkPubSub>(true, "/test")
    };
    _mav_subs = {
        std::make_shared<MavlinkPubSub>(false, "/test2")
    };

    // Open UART port

    RCLCPP_INFO(this->get_logger(), "Starting UART MAVLink Manager on serial port %s", "Test");

    // Start multithreading
    _run_threads.store(true);
    _pi_to_micro_thread = std::thread(&UartManager::pi_to_micro, this, _mav_subs);
    _micro_to_pi_thread = std::thread(&UartManager::micro_to_pi, this, _mav_pubs);
}

UartManager::~UartManager() {
    shutdown();
}

void UartManager::shutdown() {
    // Join threads
    _run_threads.store(false);
    if (_pi_to_micro_thread.joinable()) {
        _pi_to_micro_thread.join();
    }
    if (_micro_to_pi_thread.joinable()) {
        _micro_to_pi_thread.join();
    }

    // Close serial ports
}

void UartManager::pi_to_micro(std::vector<std::shared_ptr<MavlinkPubSub>> mav_subs) {
    uint64_t yo = 0;
    while (_run_threads.load()) {
        RCLCPP_INFO(this->get_logger(), "Hi %ld from pi_to_micro.", yo);
        yo += 1;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1)
        );
    }
}

void UartManager::micro_to_pi(std::vector<std::shared_ptr<MavlinkPubSub>> mav_pubs) {
    uint64_t yo = 0;
    while (_run_threads.load()) {
        RCLCPP_INFO(this->get_logger(), "Hi %ld from micro_to_pi.", yo);
        yo += 1;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1)
        );
    }
}