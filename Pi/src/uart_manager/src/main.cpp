#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "uart_manager/UartManager.h"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<UartManager> uart_manager_node = std::make_shared<UartManager>();

    // Spins until death
    rclcpp::spin(uart_manager_node);

    uart_manager_node->shutdown();

    uart_manager_node.reset();

    rclcpp::shutdown();
    return 0;
}