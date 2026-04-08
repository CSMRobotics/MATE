#include <rclcpp/rclcpp.hpp>
#include <string>
#include "std_msgs/msg/string.hpp"
#include "led_msg.pb.h"


/**
 * @brief Manages UART communication with STM32 to control LEDs
 */
class RovLEDTest : public rclcpp::Node {
public:
    /**
     * @brief Construct new object, opens UART port
     * Creates ros2 topic and subscriber
     */
    RovLEDTest();

    /**
     * @brief Destructor
     */
    ~RovLEDTest();

    /**
     * @brief Receives message from ros2 topic
     * 
     */
    void ledTopicCallback(const std_msgs::msg::String &msg);

private:
    int _serialPort = -1;
    LEDMsg _ledMsg;
    int _msgId = 0;
    struct termios _tty;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _topicSubcriber;
    
    /**
     * @brief uses nanopb to serialize message into bytes
     * 
     * @param ledOn led on or off
     * @param text text to send
     */
    std::array<pb_byte_t, LEDMsg_size> _encodeMsg(const bool &ledOn, const std::string &text);
    void _sendMsg(const bool ledOn, const std::string text);
};
