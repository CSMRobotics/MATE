#ifndef ROV_JOY_CONTROL
#define ROV_JOY_CONTROL

#include "MavlinkPubSub.hpp"
#include "mavlink/rov_interfaces/mavlink.h"
#include "mavlink/rov_interfaces/mavlink_msg_sub_controls.h"


#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

class ROVJoyControl : public rclcpp::Node {
public:
    /**
     * @brief Construct a new ROVJoyControl Node
     * 
     */
    ROVJoyControl();

private:
    //*************************************************************** */
    // Constants
    const std::string _thrustmaster_topic_name = "/thrustmaster_joy";
    const std::string _logi_topic_name = "/logi_joy";

    const std::string _mavlink_pub_name = "/sub_controls";

    //*************************************************************** */
    // Topics
    /**
     * @brief Thrustmaster flight stick topic callback
     * 
     * @param msg joystick message
     */
    void thrustmaster_callback(sensor_msgs::msg::Joy::SharedPtr msg);

    /**
     * @brief Logitech flight stick callback
     * 
     * @param msg joystick message
     */
    void logi_callback(sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _thrustmaster_sub;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _logi_sub;

    //*************************************************************** */
    // MavlinkPubSub
    mavlink_sub_controls_t _sub_controls{};
    
    std::unique_ptr<MavlinkPubSub> _mavlink_pub;
    rclcpp::TimerBase::SharedPtr _mavlink_pub_timer;
    void _mavlink_pub_timer_callback();
};

#endif // ROV_JOY_CONTROL