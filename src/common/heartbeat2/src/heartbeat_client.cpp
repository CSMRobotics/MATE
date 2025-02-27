#include <chrono>
#include <memory>
#include <future>
#include <boost/circular_buffer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "csm_common_interfaces/msg/e_stop.hpp"
#include "heartbeat_client.hpp"

HeartbeatClient::HeartbeatClient() : Node("heartbeat_client") {
    // initializing circular buffer to store request futures
    _futures = boost::circular_buffer<std::shared_future<std::shared_ptr<std_srvs::srv::Empty_Response>>>(10);

    // creating subscriber using built in time message interface because it was built in.
    handshakeClient = this->create_client<std_srvs::srv::Empty>("handshake");
    heartbeatClient = this->create_client<std_srvs::srv::Empty>("heartbeat_srv");

    // create estop publisher
    estopPub = this->create_publisher<csm_common_interfaces::msg::EStop>("estop", 10);

    // creating timers
    handshakeTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&HeartbeatClient::handshake_callback, this));
    handshakeTimer->cancel();
    pingTimer = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&HeartbeatClient::ping_callback, this));
    pingTimer->cancel();
}

void HeartbeatClient::handshake () {
    // checking if service is available or waiting for service to be availble
    RCLCPP_INFO(this->get_logger(), "Waiting for drivestation.");
    while (!handshakeClient->wait_for_service(std::chrono::milliseconds(10))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting");
            exit(1);
        }
    }
    RCLCPP_INFO(this->get_logger(), "Found drivestation");

    // Make handshake request and send
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    _handshakeFuture = std::move(handshakeClient->async_send_request(request).future.share());

    // wait for response   
    handshakeTimer->reset();
}

void HeartbeatClient::handshake_callback() {
    // check for handshake response
    if (_handshakeFuture.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        auto response = _handshakeFuture.get();
        _lastResponseTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        RCLCPP_INFO(this->get_logger(), "Connected to drivestation");
        handshakeTimer->cancel();
        pingTimer->reset();
    } 
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for response");
        exit(2);
    }
}


void HeartbeatClient::ping_callback() {
    unsigned int currentMillisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // send a request every 300ms
    if (heartbeatClient->wait_for_service(std::chrono::milliseconds(1))) {
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        _futures.push_back(std::move(heartbeatClient->async_send_request(request).future.share()));

        // debug
        // RCLCPP_INFO(this->get_logger(), "Sent ping at %u", currentMillisec);
    }
    
    // check for responses
    int futuresSize = _futures.size();
    int increment = 0;
    for (int i = 0; i < futuresSize; i++) {
        if (_futures.at(increment).wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
            // if future is ready, check if the time sent is latest, and update latest time
            _futures.erase(_futures.begin() + increment);
            _lastResponseTime = currentMillisec;
            // debug
            // RCLCPP_INFO(this->get_logger(), "Received response at: %i", currentMillisec);
        } else {
            increment += 1;
        }
    }

    // check if last response has exceeded 1000ms second, the place where disconnection happens
    if (currentMillisec - _lastResponseTime > 1000) {
        // do something is drop was detected
        RCLCPP_FATAL(this->get_logger(), "A drop has been detected, time since last ping: %ums", currentMillisec - _lastResponseTime);
        csm_common_interfaces::msg::EStop estop;
        estop.is_fatal = true;
        estopPub->publish(estop);
        exit(3);
    }
}


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<HeartbeatClient> node = std::make_shared<HeartbeatClient>();

    // start handshake to check connection to producer
    node->handshake();

    // spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}