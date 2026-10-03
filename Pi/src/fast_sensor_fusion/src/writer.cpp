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

extern "C" {
#include <mavlink/common/mavlink.h>
}

int main() {
    int fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
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

    auto* shm = static_cast<SharedMavlink*>(addr);

    // Initialize only when creating the shared-memory object.
    shm->write_index.store(0, std::memory_order_relaxed);
    shm->read_index.store(0, std::memory_order_relaxed);

    uint8_t sequence = 0;

    while (true) {
        // Create MAVLink message
        mavlink_message_t msg{};

        mavlink_heartbeat_t heartbeat{};
        heartbeat.type = MAV_TYPE_SUBMARINE;
        heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
        heartbeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        heartbeat.custom_mode = 0;
        heartbeat.system_status = MAV_STATE_ACTIVE;

        mavlink_msg_heartbeat_encode(
            1,       // system ID
            1,       // component ID
            &msg,
            &heartbeat);
        
        // Put into MAVLink shared memory buffer
        uint8_t frame[MAX_FRAME_SIZE];

        const uint16_t frame_len =
            mavlink_msg_to_send_buffer(frame, &msg);

        // Reserve a slot.
        const uint64_t write = shm->write_index.load(std::memory_order_relaxed);
        const uint64_t read = shm->read_index.load(std::memory_order_acquire);

        if (write - read >= QUEUE_SIZE) {
            std::cerr << "Shared-memory queue full! Ignoring write request.\n";
            std::this_thread::sleep_for(
                std::chrono::milliseconds(10));
            continue;
        }

        auto& slot = shm->slots[write % QUEUE_SIZE];

        std::memcpy(slot.data, frame, frame_len);
        slot.len = frame_len;

        // Publish the slot only after its contents are written.
        shm->write_index.store(
            write + 1,
            std::memory_order_release);

        std::cout << "Sent HEARTBEAT, "
                  << frame_len << " bytes\n";

        std::this_thread::sleep_for(
            std::chrono::seconds(1));

        ++sequence;
    }

    munmap(addr, sizeof(SharedMavlink));
    close(fd);
}