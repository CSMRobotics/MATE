#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

// Include MAVLink C headers
#include "mavlink/common/mavlink.h"

#define SERIAL_PORT_PATH "/dev/ttyAMA0"

int g_fd = -1;

static void open_serial_port(void) {
    g_fd = open(SERIAL_PORT_PATH, O_RDWR | O_NOCTTY | O_NDELAY);
    if (g_fd < 0) {
        perror("Failed to open " SERIAL_PORT_PATH);
        exit(EXIT_FAILURE);
    }
}

static void configure_serial_port(void) {
    struct termios tty;
    if (tcgetattr(g_fd, &tty) != 0) {
        perror("Error getting termios attributes");
        exit(EXIT_FAILURE);
    }

    // Set Baud Rate to 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Set 8N1 raw mode
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);

    tcflush(g_fd, TCIFLUSH);
    if (tcsetattr(g_fd, TCSANOW, &tty) != 0) {
        perror("Error setting termios attributes");
        exit(EXIT_FAILURE);
    }
}

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

static void transmit_mavlink(void) {
    static mavlink_message_t msg;
    static uint8_t txBuffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
			255, // system id
			1, // component id
			&msg, // message
			MAV_TYPE_GCS, // control unit
			MAV_AUTOPILOT_GENERIC,   // Autopilot type
			MAV_MODE_PREFLIGHT,      // System mode
			0,                       // Custom mode
			MAV_STATE_STANDBY        // System status
	);
    uint16_t len = mavlink_msg_to_send_buffer(txBuffer, &msg);
    ssize_t bytes_written = write(g_fd,txBuffer,len);

    if (bytes_written < 0) {
        std::cerr << "Failed to send over UART!!!" << std::endl;
    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    std::cout << "Starting UART MAVLink Manager on " << SERIAL_PORT_PATH << "...\n";

    open_serial_port();
    configure_serial_port();

    // Polling loop until CTRL+C / ROS shutdown
    while (rclcpp::ok()) {
        receive_mavlink();
        usleep(10000); // 10ms poll delay
        // transmit_mavlink();
        // usleep(10000);
    }

    if (g_fd >= 0) {
        close(g_fd);
    }

    rclcpp::shutdown();
    return 0;
}