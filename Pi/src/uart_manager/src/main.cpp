#include <signal.h>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

bool RUNNING = true;

/**
 * @brief Handles SIGINT for clean shutdown
 */
void signal_handler(int signum) {
    if (signum == SIGINT || signum == SIGTERM) {
        std::cout << "Received SIGINT, shutting down...\n";
        RUNNING = false;
    }   
}

int main(int argc, char * argv[]) {
    // ROS2 stuff
    signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);

    std::cout << "Hello World!" << std::endl;
    
    rclcpp::shutdown();
    return 0;
}
