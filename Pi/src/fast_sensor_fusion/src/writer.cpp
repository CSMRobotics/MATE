/**
 * @file writer.cpp
 * @brief Test program for publishing a MAVLink raw_IMU message over shared memory.
 */
#include "fast_sensor_fusion/MavlinkPubSub.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <chrono>

extern "C" {
#include <mavlink/common/mavlink.h>
}

const auto start_time = std::chrono::steady_clock::now();

static constexpr const char* SHM_NAME = "/mavlink_shm_sensor_fusion";

int main() {
    // Setup shared memory data structure
    MavlinkPubSub shm(true, SHM_NAME);

    while (true) {
        // Create MAVLink message
        mavlink_message_t msg{};
        
        // Create sample IMU message
        uint32_t time_sec = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time
            ).count();
        mavlink_raw_imu_t raw_imu{};
        raw_imu.id = 0;
        raw_imu.time_usec = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count();
        raw_imu.xacc = 1 + time_sec;
        raw_imu.yacc = 2 + time_sec;
        raw_imu.zacc = 98 + time_sec;
        raw_imu.xgyro = 3 + time_sec;
        raw_imu.ygyro = 4 + time_sec;
        raw_imu.zgyro = 5 + time_sec;
        raw_imu.xmag = 61 + time_sec;
        raw_imu.ymag = 62 + time_sec;
        raw_imu.zmag = 633 + time_sec;
        raw_imu.temperature = 0;

        mavlink_msg_raw_imu_encode(
            0,
            0,
            &msg,
            &raw_imu
        );

        shm.write(msg);

        std::cout << "Published Raw IMU message" << std::endl;
        
        std::this_thread::sleep_for(
        std::chrono::seconds(1));
    }
}