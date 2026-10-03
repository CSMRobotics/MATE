#include "fast_sensor_fusion/SensorFusion.h"
#include "mavlink/common/mavlink.h"

#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>

SensorFusion::SensorFusion() : rclcpp::Node("fast_sensor_fusion") {
    int result = _setup_shm();
    if (result != 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup Shared Memory.");
        exit(-1);
    }
}

//****************************************************** */
// Shared memory management

int SensorFusion::_setup_shm() {
    // Open SHM file descriptor
    _shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (_shm_fd < 0) {
        perror("shm_open");
        return -1;
    }
    // Map to local virtual address space
    void* addr = mmap(
        nullptr,
        sizeof(SharedMavlink),
        PROT_READ | PROT_WRITE,
        MAP_SHARED,
        _shm_fd,
        0
    );

    if (addr == MAP_FAILED) {
        perror("mmap");
        return -1;
    }

    // Store the pointer to Shared Memory
    _mav_shm = static_cast<SharedMavlink*>(addr);

    return 0;
}

int SensorFusion::_close_shm() {
    munmap((void*) _mav_shm, sizeof(SharedMavlink));
    int result = close(_shm_fd);
    if (result < 0) {
        std::cerr << "close() shared memory FD failed: " << " (" << strerror(errno) << ")" << std::endl;
    }
}

int SensorFusion::_update_imu_data() {
    mavlink_message_t message{};
    mavlink_status_t status{};

    // Get current write and read indexes
    _shm_read_idx = _mav_shm->read_index.load(std::memory_order_relaxed);
    uint64_t shm_write_idx = _mav_shm->write_index.load(std::memory_order_acquire);

    // See if new message are available
    if (_shm_read_idx == shm_write_idx) {
        RCLCPP_INFO(this->get_logger(), "No message available.");
        return 0;
    }

    // Read new message
    SharedMavlink::Slot& read_slot = _mav_shm->slots[_shm_read_idx % QUEUE_SIZE];

    for (uint32_t i = 0; i < read_slot.len; i++) {
        if (mavlink_parse_char(
            MAVLINK_COMM_0,
            read_slot.data[i],
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
        } else {
            RCLCPP_WARN(this->get_logger(), "mavlink_parse_char() had an error");
        }
    }

    // Iterate read pointer
    _shm_read_idx++;
    _mav_shm->read_index.store(
        _shm_read_idx,
        std::memory_order_release);
}