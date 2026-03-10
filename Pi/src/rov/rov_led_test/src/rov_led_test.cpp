#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp" // ros2 cpp library

#define PACKETSIZE sizeof(LED_Msg)

// Message data to send to STM32
struct LED_Msg {
    bool ledOn = false;
    int number = 67;
};

/**
 * Supports bool, int
 */
class RovLEDTest : public rclcpp::Node {
public:
    RovLEDTest() : Node("rov_led_test") {
        RCLCPP_INFO(this->get_logger(), "RovLEDTest node has been started.");
        
        // Access UART port
        int serial_port = open("/dev/ttyAMA0", O_RDWR);
        if (serial_port < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
        }
    }
private:
    LED_Msg _ledMsg;

    // Turn LED_Msg into serial data to send over UART
    void _serializeMsg() {
        uint8_t buffer[PACKETSIZE];
        for (int i = 0; i < _ledMsg.NUM_ELEMENTS; i++) {
            
        }
        buffer[0] = _ledMsg.ledOn ? 1 : 0;
        
    }
};



int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RovLEDTest>());
    rclcpp::shutdown();
    return 0;
}
