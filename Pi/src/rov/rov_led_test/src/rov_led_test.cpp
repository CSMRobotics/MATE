#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <array>

#include "rclcpp/rclcpp.hpp" // ros2 cpp library
#include "rov_led_test.hpp"

// nanopb headers
#include "led_msg.pb.h"
#include "pb_encode.h"


RovLEDTest::RovLEDTest() : Node("rov_led_test") {
    RCLCPP_INFO(this->get_logger(), "RovLEDTest node has been started.");

    // Create topic subscriber
    _topicSubcriber = this->create_subscription<std_msgs::msg::String>("LED_Test", 10, std::bind(&RovLEDTest::ledTopicCallback, this, std::placeholders::_1));
    
    // Followed this tutorial for UART https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
    // Open UART port
    _serialPort = open("/dev/ttyAMA0", O_RDWR);
    if (_serialPort < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error %i from opening UART port: %s", errno, strerror(errno));
        exit(1);
    }

    // Configure UART port
    if(tcgetattr(_serialPort, &_tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
        exit(2);
    }
    _tty.c_cflag &= ~PARENB;        // Disable parity bit
    _tty.c_cflag &= ~CSTOPB;        // Sets stop bit to 1 instead of 2
    _tty.c_cflag &= ~CSIZE;         // Clear current byte size setting
    _tty.c_cflag |= CS8;            // Sets byte size to 8 bits
    _tty.c_cflag &= ~CRTSCTS;       // Disables hardware flow control
    _tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    _tty.c_lflag &= ~ICANON;        // Disable canonical mode, idk what that is
    _tty.c_lflag &= ~ECHO;          // Disable echo
    _tty.c_lflag &= ~ECHOE;         // Disable erasure
    _tty.c_lflag &= ~ECHONL;        // Disable new-line echo
    _tty.c_lflag &= ~ISIG;          // Disable interpretation of INTR, QUIT and SUSP signals
    _tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    _tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special processing of raw data
    _tty.c_oflag &= ~OPOST;         // Prevent special interpretation of output bytes (e.g. newline chars)
    _tty.c_oflag &= ~ONLCR;         // Prevent conversion of newline to carriage return/line feed
    _tty.c_cc[VTIME] = 10;          // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    _tty.c_cc[VMIN] = LEDMsg_size;  // Wait until entire message is read by read()
    // Set baud rate
    cfsetispeed(&_tty, B115200);
    cfsetospeed(&_tty, B115200);
    // Save tty settings, also checking for error
    if (tcsetattr(_serialPort, TCSANOW, &_tty) != 0) {
        RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }

    RCLCPP_INFO(this->get_logger(), "UART Port configured");
}

RovLEDTest::~RovLEDTest() {
    if (_serialPort >= 0) {
        close(_serialPort);
    }
}

void RovLEDTest::ledTopicCallback(const std_msgs::msg::String &msg) {
    RCLCPP_INFO(this->get_logger(), "Received data: %s \n", msg.data.c_str());
    bool ledOn = false;
    std::string text = msg.data.substr(2);
    if (msg.data.substr(0, 2) == "on") {
        ledOn = true;
    }
    _sendMsg(ledOn, text);
}

std::array<pb_byte_t, LEDMsg_size> RovLEDTest::_encodeMsg(const bool &ledOn, const std::string &text) {
    // Build the message
    std::array<pb_byte_t, LEDMsg_size> buffer{};  // Contains encoded message afterwards
    _ledMsg.id = _msgId;
    _ledMsg.ledOn = ledOn;
    strncpy(_ledMsg.text, text.c_str(), sizeof(_ledMsg.text));

    // Create nanopb stream
    pb_ostream_t stream = pb_ostream_from_buffer(buffer.data(), buffer.size());

    // Encode the mmessage
    bool encodeSuccess = pb_encode(&stream, &LEDMsg_msg, &_ledMsg);
    if (!encodeSuccess) {
        RCLCPP_ERROR(this->get_logger(), "Message encode failed. id = %i, ledOn = %i, text: %s", _ledMsg.id, _ledMsg.ledOn, _ledMsg.text);
    }
    return buffer;
}

void RovLEDTest::_sendMsg(const bool ledOn, const std::string text) {
    std::array<pb_byte_t, LEDMsg_size> encodedMsg = _encodeMsg(ledOn, text);
    write(_serialPort, encodedMsg.data(), encodedMsg.size());
    std::cout << "Size: " << encodedMsg.size() << std::endl;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RovLEDTest>());
    rclcpp::shutdown();
    return 0;
}
