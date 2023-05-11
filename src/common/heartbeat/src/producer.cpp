#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <csm_common_interfaces/srv/handshake.hpp>

#include "heartbeat/date.h"

#include <chrono>
#include <iostream>
#include <sstream>
#include <random>

class Producer : public rclcpp::Node {
public:
    Producer() : Node("heartbeat_producer") {
        using namespace std::chrono_literals;
        heartbeat_service_listener = this->create_service<csm_common_interfaces::srv::Handshake>("heartbeat_handshake", std::bind(&Producer::handshake, this, std::placeholders::_1, std::placeholders::_2));
        heartbeat_pub = this->create_publisher<builtin_interfaces::msg::Time>("heartbeat", 10);
        srand(time(NULL));
    }
private:
    void handshake(const csm_common_interfaces::srv::Handshake::Request::SharedPtr req,
            csm_common_interfaces::srv::Handshake::Response::SharedPtr res) {
        using namespace date; // i use date library to make outputting chrono::time_point not hell (not necessary in c++20)

        // acknowledge reception of message
        const builtin_interfaces::msg::Time pt = rclcpp::Node::get_clock()->now();
        std::stringstream ss;
        ss.clear();
        ss << std::chrono::system_clock::time_point{std::chrono::seconds{pt.sec} + std::chrono::nanoseconds{pt.nanosec}};
        RCLCPP_INFO(this->get_logger(), "Received start request at %s UTC", ss.str().c_str());
        res->sec = pt.sec;
        res->nanosec = pt.nanosec;
        // create heartbeat timer to publish a heartbeat ever req->period milliseconds
        heartbeat_timer = this->create_wall_timer(std::chrono::milliseconds(req->period), std::bind(&Producer::heartbeat, this));
    }

    void heartbeat() {
        using namespace date; // i use date library to make outputting chrono::time_point not hell (not necessary in c++20)

        const builtin_interfaces::msg::Time stamp = this->get_clock()->now();
        std::stringstream ss;
        ss.clear();
        ss << std::chrono::system_clock::time_point{std::chrono::seconds{stamp.sec} + std::chrono::nanoseconds{stamp.nanosec}};
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), "Beat at %s UTC", ss.str().c_str());
#endif
        heartbeat_pub->publish(stamp);
    }

    rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr heartbeat_pub;
    rclcpp::TimerBase::SharedPtr heartbeat_timer;
    rclcpp::Service<csm_common_interfaces::srv::Handshake>::SharedPtr heartbeat_service_listener;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    // rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(std::make_shared)
    rclcpp::spin(std::make_shared<Producer>());

    rclcpp::shutdown();
}