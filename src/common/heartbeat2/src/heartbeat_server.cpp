#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "csm_common_interfaces/msg/e_stop.hpp"
#include "heartbeat_server.hpp"

HeartbeatServer::HeartbeatServer() : Node("heartbeat_producer") {
    // service to receive pings
    heartbeatSrv = this->create_service<std_srvs::srv::Empty>("heartbeat_srv", std::bind(&HeartbeatServer::heartbeat_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Setting up handshake
    RCLCPP_INFO(this->get_logger(), "Waiting for ROV");
    handshakeSrv = this->create_service<std_srvs::srv::Empty>("handshake", std::bind(&HeartbeatServer::handshake, this, std::placeholders::_1, std::placeholders::_2));

    // create timers
    watchdog = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&HeartbeatServer::watchdog_callback, this));
    watchdog->cancel();

    // create estop publisher
    estopPub = this->create_publisher<csm_common_interfaces::msg::EStop>("estop", 10);
}

void HeartbeatServer::handshake(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) {
    // send response to handshake request to start heartbeat
    unsigned int currentMillisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    _lastPingTime = currentMillisec;
    RCLCPP_INFO(this->get_logger(), "Connected to ROV");

    // initialize the timer to watch the incoming pings, check if ROV has been momentarily disconnected and reconnected
    if (watchdog->is_canceled() == true) {
        watchdog->reset();
    } else {
        RCLCPP_WARN(this->get_logger(), "A drop was detected, connection has been restablished");
    }
}

    
void HeartbeatServer::watchdog_callback() {
    // check to see if the time since last ping received exceeds 1000ms.
    unsigned int currentMillisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    if (currentMillisec > _lastPingTime && currentMillisec - _lastPingTime > 1000) {
        // do something if drop was detected
        RCLCPP_FATAL(this->get_logger(), "A drop has been detected, time since last ping: %ims", currentMillisec - _lastPingTime);
        csm_common_interfaces::msg::EStop estop;
        estop.is_fatal = true;
        estopPub->publish(estop);
        watchdog->cancel();
        exit(3);
    }
}

void HeartbeatServer::heartbeat_callback(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response) {
    // check if any previous connection has been made
    if (watchdog->is_canceled() == true) {
        RCLCPP_INFO(this->get_logger(), "Connected to ROV");
        watchdog->reset();
    }

    // Receive request and log time request received
    unsigned int currentMillisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    _lastPingTime = currentMillisec;

    // debug
    // RCLCPP_INFO(this->get_logger(), "Received ping at %i", currentMillisec);
}


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HeartbeatServer>());
    rclcpp::shutdown();
    return 0;
}

