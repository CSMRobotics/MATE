#include <atomic>
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

#include "fast_sensor_fusion/SharedMavlink.h"

int main() {
    // Open shared memory file descriptor
    int fd = shm_open(SHM_NAME, O_RDWR, 0666);
    if (fd < 0) {
        perror("shm_open");
        return 1;
    }
    // Map to local virtual address space
    void* addr = mmap(
        nullptr,
        sizeof(SharedMavlink),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        fd,
        0);

    if (addr == MAP_FAILED) {
        perror("mmap");
        return -1;
    }
    
    SharedMavlink* shm = static_cast<SharedMavlink*>(addr);

    mavlink_message_t message{};
    mavlink_status_t status{};

    uint64_t read_idx =
        shm->read_index.load(std::memory_order_relaxed);

    while (true) {
        const uint64_t write_idx =
            shm->write_index.load(std::memory_order_acquire);

        if (read_idx == write_idx) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
            continue;
        }

        auto& slot = shm->slots[read_idx % QUEUE_SIZE];

        for (uint32_t i = 0; i < slot.len; ++i) {
            if (mavlink_parse_char(
                    MAVLINK_COMM_0,
                    slot.data[i],
                    &message,
                    &status)) {

                std::cout
                    << "Received MAVLink message: "
                    << "msgid=" << message.msgid
                    << " sysid="
                    << static_cast<int>(message.sysid)
                    << " compid="
                    << static_cast<int>(message.compid)
                    << '\n';

                if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t heartbeat{};

                    mavlink_msg_heartbeat_decode(
                        &message,
                        &heartbeat);

                    std::cout
                        << "  HEARTBEAT type="
                        << static_cast<int>(heartbeat.type)
                        << " autopilot="
                        << static_cast<int>(heartbeat.autopilot)
                        << '\n';
                }
            }
        }

        ++read_idx;

        // Release the slot to the producer.
        shm->read_index.store(
            read_idx,
            std::memory_order_release);
    }

    munmap(addr, sizeof(SharedMavlink));
    close(fd);
}
