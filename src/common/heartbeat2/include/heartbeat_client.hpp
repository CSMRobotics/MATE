#ifndef HEARTBEAT_CLIENT_HPP
#define HEARTBEAT_CLIENT_HPP

#include <rclcpp/rclcpp.hpp>
#include <boost/circular_buffer.hpp>
#include <std_srvs/srv/empty.hpp>

#include "csm_common_interfaces/msg/e_stop.hpp"

class HeartbeatClient : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Heartbeat Client object
         * 
         */
        HeartbeatClient();

        /**
         * @brief Establishes connection to heartbeat server
         * 
         */
        void handshake();
    
    private:
        /**
         * @brief Checks for handshake response
         * 
         */
        void handshake_callback();

        /**
         * @brief Sends requests to server and checks if responses are sent
         * 
         */
        void ping_callback();
    
    
        // Time since last ping was received
        unsigned int _lastResponseTime;
        boost::circular_buffer<std::shared_future<std::shared_ptr<std_srvs::srv::Empty_Response>>> _futures;
    
        // heartbeat and handshake
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr handshakeClient;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr heartbeatClient;
    
        // storing handshake futures and setting up timer to send requests and check for responses
        std::shared_future<std::shared_ptr<std_srvs::srv::Empty_Response>> _handshakeFuture;
        rclcpp::TimerBase::SharedPtr handshakeTimer;
        rclcpp::TimerBase::SharedPtr pingTimer;
    
        // the estop
        rclcpp::Publisher<csm_common_interfaces::msg::EStop>::SharedPtr estopPub;
};

#endif // (HEARTBEAT_CLIENT_H)