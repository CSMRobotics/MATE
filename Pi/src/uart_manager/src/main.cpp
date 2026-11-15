#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "uart_manager/UartManager.h"

static void receive_mavlink(void) {
    uint8_t rx_buffer[256];
    ssize_t bytes_read = read(g_fd, rx_buffer, sizeof(rx_buffer));

    if (bytes_read > 0) {
        for (ssize_t i = 0; i < bytes_read; ++i) {
            mavlink_message_t msg;
            mavlink_status_t status;

            // Stream parsing byte-by-byte
            if (mavlink_parse_char(MAVLINK_COMM_0, rx_buffer[i], &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                    std::cout << "[SUCCESS] Received HEARTBEAT | "
                              << "SysID: " << (int)msg.sysid
                              << " | CompID: " << (int)msg.compid
                              << " | Type: " << (int)heartbeat.type << std::endl;
                }
            }
        }
    }
}


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