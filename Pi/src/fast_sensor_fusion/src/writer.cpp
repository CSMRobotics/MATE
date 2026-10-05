/**
 * @file writer.cpp
 * @brief Test program for publishing a MAVLink raw_IMU message over shared memory.
 */
#include "fast_sensor_fusion/SharedMavlink.h"

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

int fd = 0;
SharedMavlink* shm;

int setup_shm() {
    fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        perror("shm_open");
        return 1;
    }

    if (ftruncate(fd, sizeof(SharedMavlink)) < 0) {
        perror("ftruncate");
        return 1;
    }

    void* addr = mmap(
        nullptr,
        sizeof(SharedMavlink),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        0);

    if (addr == MAP_FAILED) {
        perror("mmap");
        return 1;
    }

    shm = static_cast<SharedMavlink*>(addr);

    // Initialize only when creating the shared-memory object.
    shm->write_index.store(0, std::memory_order_relaxed);

    return 0;
}

void write_mavlink_to_shm(mavlink_message_t &msg) {
    // Put into MAVLink shared memory buffer
    uint8_t frame[MAX_FRAME_SIZE];

    const uint16_t frame_len =
        mavlink_msg_to_send_buffer(frame, &msg);

    // Reserve a slot.
    const uint64_t write = shm->write_index.load(std::memory_order_relaxed);

    auto& slot = shm->slots[write % QUEUE_SIZE];

    std::memcpy(slot.data, frame, frame_len);
    slot.len = frame_len;

    // Publish the slot only after its contents are written.
    shm->write_index.store(
        write + 1,
        std::memory_order_release);

    std::cout << "Sent IMU message, "
                << frame_len << " bytes\n";
}

int main() {
    // Setup shared memory data structure
    if (setup_shm() != 0) {
        return -1;
    }

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

        write_mavlink_to_shm(msg);

        std::this_thread::sleep_for(
        std::chrono::seconds(1));
    }

    munmap(shm, sizeof(SharedMavlink));
    close(fd);
}