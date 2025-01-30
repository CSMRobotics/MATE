#include <chrono>
#include <memory>
#include <future>
#include <boost/circular_buffer.hpp>
#include <rclcpp/rclcpp.hpp>

#include "csm_common_interfaces/srv/heartbeat.hpp"
#include "csm_common_interfaces/msg/e_stop.hpp"

class Client : public rclcpp::Node {
public:
    Client() : Node("heartbeat_client") {
        // initializing circular buffer to store request futures
        _futures = new boost::circular_buffer<std::shared_future<std::shared_ptr<csm_common_interfaces::srv::Heartbeat_Response>>>(10);

        // creating subscriber using built in time message interface because it was built in.
        handshakeClient = this->create_client<csm_common_interfaces::srv::Heartbeat>("handshake");
        heartbeatClient = this->create_client<csm_common_interfaces::srv::Heartbeat>("heartbeat_srv");

        // create estop publisher
        estopPub = this->create_publisher<csm_common_interfaces::msg::EStop>("estop", 10);

        // creating timers
        handshakeTimer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Client::handshake_callback, this));
        handshakeTimer->cancel();
        pingTimer = this->create_wall_timer(std::chrono::milliseconds(300), std::bind(&Client::ping_callback, this));
        pingTimer->cancel();
    }

    void handshake () {
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
        auto request = std::make_shared<csm_common_interfaces::srv::Heartbeat::Request>();
        request->milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        _handshakeFuture = std::move(handshakeClient->async_send_request(request).future.share());

        // wait for response   
        handshakeTimer->reset();
    }

private:
    void handshake_callback() {
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


    void ping_callback() {
        unsigned int currentMillisec = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // send a request every 300ms
        if (heartbeatClient->wait_for_service(std::chrono::milliseconds(1))) {
            auto request = std::make_shared<csm_common_interfaces::srv::Heartbeat::Request>();
            request->milliseconds = currentMillisec;
            _futures->push_back(std::move(heartbeatClient->async_send_request(request).future.share()));

            // debug
            // RCLCPP_INFO(this->get_logger(), "Sent ping at %u", currentMillisec);
        }
        
        // check for responses
        int futuresSize = _futures->size();
        int increment = 0;
        std::shared_ptr<csm_common_interfaces::srv::Heartbeat::Response> response;
        for (int i = 0; i < futuresSize; i++) {
            if (_futures->at(increment).wait_for(std::chrono::milliseconds(1)) == std::future_status::ready) {
                // if future is ready, check if the time sent is latest, and update latest time
                response = _futures->at(increment).get();
                _futures->erase(_futures->begin() + increment);
                _lastResponseTime = currentMillisec;
                // debug
                // RCLCPP_INFO(this->get_logger(), "Received response at: %i", int(response->milliseconds));
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


    // Time since last ping was received
    unsigned int _lastResponseTime;
    boost::circular_buffer<std::shared_future<std::shared_ptr<csm_common_interfaces::srv::Heartbeat_Response>>>* _futures;

    // heartbeat and handshake
    rclcpp::Client<csm_common_interfaces::srv::Heartbeat>::SharedPtr handshakeClient;
    rclcpp::Client<csm_common_interfaces::srv::Heartbeat>::SharedPtr heartbeatClient;

    // storing handshake futures and setting up timer to send requests and check for responses
    std::shared_future<std::shared_ptr<csm_common_interfaces::srv::Heartbeat_Response>> _handshakeFuture;
    rclcpp::TimerBase::SharedPtr handshakeTimer;
    rclcpp::TimerBase::SharedPtr pingTimer;

    // the estop
    rclcpp::Publisher<csm_common_interfaces::msg::EStop>::SharedPtr estopPub;
};


int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<Client> node = std::make_shared<Client>();

    // start handshake to check connection to producer
    node->handshake();

    // spin the node
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}