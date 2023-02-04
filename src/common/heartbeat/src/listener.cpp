#include <iostream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/empty.hpp>

#include "common_interfaces/msg/e_stop.hpp"
#include "common_interfaces/srv/handshake.hpp"
#include "common_interfaces/srv/heartbeat_control.hpp"
#include "heartbeat/date.h"

namespace {
    void declare_params(rclcpp::Node* Node) {
        Node->declare_parameter("period_ms", 3000); // how long should the time between pings be? (negotiated during handshake)
        Node->declare_parameter("timeout_ms", 3500); // how long is too long between pings?
        Node->declare_parameter("watchdog_ms", 100); // how often should the watchdog check time between pings?
    }
}

class Listener : public rclcpp::Node {
public:
    Listener() : Node("heartbeat_listener") {
        declare_params(this);
        using namespace std::chrono_literals;
        // create handshake client and control service
        handshake_client = this->create_client<common_interfaces::srv::Handshake>("heartbeat_handshake");
        control_service = this->create_service<common_interfaces::srv::HeartbeatControl>("heartbeat_control", std::bind(&Listener::heartbeat_control, this, std::placeholders::_1, std::placeholders::_2));

        // create subscribers to handle incoming messages
        heartbeat_sub = this->create_subscription<builtin_interfaces::msg::Time>("heartbeat", 10, std::bind(&Listener::heartbeat_callback, this, std::placeholders::_1));

        // create estop publisher
        estop_pub = this->create_publisher<common_interfaces::msg::EStop>("estop", 10);

        // request producer's start
        handshake();

        // create timeout watchdog
        watchdog_timer = this->create_wall_timer(std::chrono::milliseconds(this->get_parameter("watchdog_ms").as_int()), std::bind(&Listener::watchdog, this));
    }
private:
    // send request for heartbeat to the heartbeat producer
    void handshake() {
        using namespace date;
        auto req = std::make_shared<common_interfaces::srv::Handshake::Request>();

        // wait for service to be available
        while(!handshake_client->wait_for_service(std::chrono::milliseconds(100))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the <%s> service. Exiting.", handshake_client->get_service_name());
                exit(1);
            }
            RCLCPP_INFO_THROTTLE(rclcpp::get_logger("rclcpp"), *this->get_clock(), 100, "<%s> Service not available, waiting again...", handshake_client->get_service_name());
        }

        // stamp the request
        const std::chrono::system_clock::time_point pt = std::chrono::system_clock::now();
        auto pt_sec = std::chrono::time_point_cast<std::chrono::seconds>(pt);
        req->sec = pt_sec.time_since_epoch().count();
        req->nanosec = (pt - pt_sec).count();
        req->period = this->get_parameter("period_ms").as_int();

        // callback to activate the heartbeat watchdog
        auto response_received_callback = [this](rclcpp::Client<common_interfaces::srv::Handshake>::SharedFuture future) {
            auto res = future.get();
            std::stringstream ss;
            ss.clear();
            this->last_received_ping = std::chrono::system_clock::time_point{std::chrono::seconds{res->sec} + std::chrono::nanoseconds{res->nanosec}};
            ss << this->last_received_ping;
            RCLCPP_INFO(this->get_logger(), "Received handshake response at %s UTC", ss.str().c_str());
            this->watchdog_active = true;
        };
        std::stringstream ss;
        ss.clear();
        ss << pt;
        RCLCPP_INFO(this->get_logger(), "Sent handshake request at %s UTC", ss.str().c_str());
        // send the request
        auto res_future = handshake_client->async_send_request(req, response_received_callback);
    }

    // subscription callback
    void heartbeat_callback(const builtin_interfaces::msg::Time::SharedPtr msg) {
        using namespace date; // i use date library to make outputting chrono::time_point not hell
        std::chrono::system_clock::time_point msg_recv = std::chrono::system_clock::time_point{std::chrono::seconds{msg->sec} + std::chrono::nanoseconds{msg->nanosec}};
        // only accept most recent packets if they arrive out-of-order
        if(msg_recv > last_received_ping) {
#ifdef DEBUG
            std::stringstream ss;
            ss.clear();
            ss << msg_recv;
            RCLCPP_INFO(this->get_logger(), "Received ping at %s UTC", ss.str().c_str());
            RCLCPP_INFO(this->get_logger(), "Time since last ping %lu milliseconds", std::chrono::duration_cast<std::chrono::milliseconds>(msg_recv - last_received_ping).count());
#endif
            last_received_ping = msg_recv;
        }
    }

    // service callback that will restart handshake
    void heartbeat_control(const common_interfaces::srv::HeartbeatControl::Request::SharedPtr req, common_interfaces::srv::HeartbeatControl::Response::SharedPtr res __attribute__((unused))) {
        if(req->restart) {
            watchdog_active = false;
            heartbeat_sub.reset();
            handshake();
            heartbeat_sub = this->create_subscription<builtin_interfaces::msg::Time>("heartbeat", 10, std::bind(&Listener::heartbeat_callback, this, std::placeholders::_1));
        } else if (req->stop) {
            // TODO: do we get hanging weak ptrs?
            watchdog_active = false;
            heartbeat_sub.reset();
        }
    }

    // timeout watchdog that will cause fatal estop if connection interrupted
    void watchdog() {
        if(watchdog_active && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_received_ping).count() > this->get_parameter("timeout_ms").as_int()) {
            RCLCPP_FATAL(this->get_logger(), "%s watchdog detected a time between heartbeats >%lums", this->get_name(), this->get_parameter("timeout_ms").as_int());
            watchdog_active = false;
            heartbeat_sub.reset();
            common_interfaces::msg::EStop estop;
            estop.is_fatal = true;
            estop_pub->publish(estop);
        }
    }

    std::chrono::system_clock::time_point last_received_ping;
    rclcpp::TimerBase::SharedPtr watchdog_timer;
    bool watchdog_active = false;
    rclcpp::Publisher<common_interfaces::msg::EStop>::SharedPtr estop_pub;
    rclcpp::Subscription<builtin_interfaces::msg::Time>::SharedPtr heartbeat_sub;
    rclcpp::Client<common_interfaces::srv::Handshake>::SharedPtr handshake_client;
    rclcpp::Service<common_interfaces::srv::HeartbeatControl>::SharedPtr control_service;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<Listener>());
    // auto node = std::make_shared<Listener>();
    // rclcpp::executors::MultiThreadedExecutor exec;
    // exec.add_node(node);
    // exec.spin();

    rclcpp::shutdown();
}