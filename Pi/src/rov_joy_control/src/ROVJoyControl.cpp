#include "rov_joy_control/ROVJoyControl.h"
#include "MavlinkPubSub.hpp"

ROVJoyControl::ROVJoyControl() : Node("rov_joy_control") {
    // Setup topics
    _thrustmaster_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/thrustmaster_joy",
        10,
        std::bind(&ROVJoyControl::thrustmaster_callback, this, std::placeholders::_1)
    );

    _logi_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/logi_joy",
        10,
        std::bind(&ROVJoyControl::logi_callback, this, std::placeholders::_1)
    );

    // Setup Mavlink Publisher
    try {
        _mavlink_pub = std::make_unique<MavlinkPubSub>(true, _mavlink_pub_name);
        RCLCPP_INFO(this->get_logger(), "Created mavlink pub channel");
    
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create mavlink publisher: %s", e.what());
        rclcpp::shutdown();
        return;
    }
    
    _mavlink_pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&ROVJoyControl::_mavlink_pub_timer_callback, this)
    );
}

void ROVJoyControl::thrustmaster_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    _sub_controls.velocity_x = msg->axes[1];
    _sub_controls.velocity_y = -msg->axes[0];
    _sub_controls.velocity_z = -msg->axes[2];

    _sub_controls.estop = msg->buttons[1];
    _sub_controls.mode_switch = msg->buttons[3];
}

void ROVJoyControl::logi_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    _sub_controls.omega_x = -msg->axes[0];
    _sub_controls.omega_y = -msg->axes[1];
    _sub_controls.omega_z = -msg->axes[2];
}

void ROVJoyControl::_mavlink_pub_timer_callback() {
    mavlink_message_t msg{};
    mavlink_msg_sub_controls_encode(
        0,
        0,
        &msg,
        &_sub_controls
    );
    int write_result = _mavlink_pub->write(msg);

    // DEBUG
    if (write_result == -1) {
        RCLCPP_WARN(this->get_logger(), "Queue is full %d", write_result);
    } else if (write_result != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to publish to mavlink channel %d", write_result);
    } else {
        RCLCPP_INFO(this->get_logger(), "Published to mavlink");
    }
}