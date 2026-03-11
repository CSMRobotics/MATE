#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp" // ros2 cpp library
#include "rov_led_test.hpp"

#define PACKETSIZE sizeof(LED_Msg)

RovLEDTest::RovLEDTest() : Node("rov_led_test") {
    RCLCPP_INFO(this->get_logger(), "RovLEDTest node has been started.");
    
    // Open UART port
    int serial_port = open("/dev/ttyAMA0", O_RDWR);
    if (serial_port < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error %i from opening UART port: %s", errno, strerror(errno));
        exit(1);
    }
}

RovLEDTest::~RovLEDTest() {
    if (_serialPort >= 0) {
        close(_serialPort);
    }
}



int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RovLEDTest>());
    rclcpp::shutdown();
    return 0;
}
