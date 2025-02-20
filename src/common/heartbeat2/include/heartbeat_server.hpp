#ifndef HEARTBEAT_SERVER_HPP
#define HEARTBEAT_SERVER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "csm_common_interfaces/msg/e_stop.hpp"

class HeartbeatServer : public rclcpp::Node {
    public:
        /**
         * @brief Construct a new Heartbeat Server object
         * 
         */
        HeartbeatServer();
    
    private:
        /**
         * @brief Establishes connection with heartbeat client
         * 
         * @param request empty request
         * @param response empty response
         */
        void handshake(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);
    
        /**
         * @brief Watches if client stops sending requests
         * 
         */
        void watchdog_callback();

        /**
         * @brief Processes requests from client
         * 
         * @param request empty request
         * @param response empty response
         */
        void heartbeat_callback(std_srvs::srv::Empty::Request::SharedPtr request, std_srvs::srv::Empty::Response::SharedPtr response);
    
        // last time request was received
        unsigned int _lastPingTime;

        // handshake and heartbeat
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr handshakeSrv;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr heartbeatSrv;
    
        // timer to periodically check for pings and send responses
        rclcpp::TimerBase::SharedPtr watchdog;
    
        // estop
        rclcpp::Publisher<csm_common_interfaces::msg::EStop>::SharedPtr estopPub;
};


#endif // (HEARTBEAT_SERVER_H)