/**
 * @file writer.cpp
 * @brief Test program for publishing a MAVLink raw_IMU message over shared memory.
 */
#include "MavlinkPubSub.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include <chrono>

#include <mavlink/rov_interfaces/mavlink.h>

const auto start_time = std::chrono::steady_clock::now();

static constexpr const char* SHM_NAME = "/mavlink_test_heartbeat";

int main() {
    // Setup shared memory data structure
    MavlinkPubSub shm(false, SHM_NAME);

    while (true) {
        mavlink_message_t msg;
        mavlink_status_t status;

        if (shm.read(msg, status) == 1) {
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                std::cout << "[SUCCESS] Received HEARTBEAT | "
                            << "SysID: " << (int)msg.sysid
                            << " | CompID: " << (int)msg.compid
                            << " | Type: " << (int)heartbeat.type << std::endl;
            } else {
                std::cout << "[ERROR] Received unknown message with id: " << msg.msgid << std::endl;
            }
        }
    }
}