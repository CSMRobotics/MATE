#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "csm_common_interfaces/msg/e_stop.hpp"
#include "csm_common_interfaces/srv/heartbeat.hpp"

class Server : public rclcpp::Node {
public:
    Server() : Node("heartbeat_producer") {
        // service to receive pings
        heartbeatSrv = this->create_service<csm_common_interfaces::srv::Heartbeat>("heartbeat_srv", std::bind(&Server::heartbeat_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Setting up handshake
        RCLCPP_INFO(this->get_logger(), "Waiting for ROV");
        handshakeSrv = this->create_service<csm_common_interfaces::srv::Heartbeat>("handshake", std::bind(&Server::handshake, this, std::placeholders::_1, std::placeholders::_2));

        // create timers
        watchdog = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&Server::watchdog_callback, this));
        watchdog->cancel();

        // create estop publisher
        estopPub = this->create_publisher<csm_common_interfaces::msg::EStop>("estop", 10);
    }

private:
    void handshake(csm_common_interfaces::srv::Heartbeat::Request::SharedPtr request, csm_common_interfaces::srv::Heartbeat::Response::SharedPtr response) {
        // send response to handshake request to start heartbeat
        unsigned int millisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        response->milliseconds = request->milliseconds;
        _lastPingTime = millisec;
        RCLCPP_INFO(this->get_logger(), "Connected to ROV");

        // initialize the timer to watch the incoming pings, check if ROV has been momentarily disconnected and reconnected
        if (watchdog->is_canceled() == true) {
            watchdog->reset();
        } else {
            RCLCPP_WARN(this->get_logger(), "A drop was detected, connection has been restablished");
        }
    }

    
    void watchdog_callback() {
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

    void heartbeat_callback(csm_common_interfaces::srv::Heartbeat::Request::SharedPtr request, csm_common_interfaces::srv::Heartbeat::Response::SharedPtr response) {
        // check if any previous connection has been made
        if (watchdog->is_canceled() == true) {
            RCLCPP_INFO(this->get_logger(), "Connected to ROV");
            watchdog->reset();
        }

        // Receive request and log request
        unsigned int currentMillisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        _lastPingTime = currentMillisec;

        // debug
        // RCLCPP_INFO(this->get_logger(), "Received ping at %i  --- Time Elapsed: %i", int(request->milliseconds), int(currentMillisec - request->milliseconds));
        
        // send request time back to client
        response->milliseconds = request->milliseconds;
    }

    
    unsigned int _lastPingTime;
    csm_common_interfaces::srv::Heartbeat::Request _lastPing;
    rclcpp::Service<csm_common_interfaces::srv::Heartbeat>::SharedPtr handshakeSrv;
    rclcpp::Service<csm_common_interfaces::srv::Heartbeat>::SharedPtr heartbeatSrv;

    // timer to periodically check for pings and send responses
    rclcpp::TimerBase::SharedPtr watchdog;

    // estop
    rclcpp::Publisher<csm_common_interfaces::msg::EStop>::SharedPtr estopPub;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Server>());
    rclcpp::shutdown();
    return 0;
}

