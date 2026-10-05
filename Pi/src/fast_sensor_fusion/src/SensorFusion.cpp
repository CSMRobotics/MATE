#include "fast_sensor_fusion/SensorFusion.h"
#include "mavlink/common/mavlink.h"

#include <iostream>
#include <sys/mman.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <cstdint>

SensorFusion::SensorFusion() : rclcpp::Node("fast_sensor_fusion") {
    // Setup shared memory
    int result = _setup_shm();
    if (result != 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to setup Shared Memory.");
        exit(-1);
    }
}

void SensorFusion::update() {
    // Get time to delay for next loop iteration
    timespec start_time{};
    clock_gettime(CLOCK_MONOTONIC, &start_time);

    // The main loop
    this->_update_imu_data();

    // Delay until next call
    // Advance by exactly one period.
    timespec next_time = start_time;
    next_time.tv_nsec += LOOP_PERIOD_NS;

    while (next_time.tv_nsec >= 1'000'000'000) {
        next_time.tv_nsec -= 1'000'000'000;
        ++next_time.tv_sec;
    }

    // Sleep until the next time
    clock_nanosleep(
        CLOCK_MONOTONIC,
        TIMER_ABSTIME,
        &next_time,
        nullptr);
}

void SensorFusion::shutdown() {
    _close_shm();
}

//****************************************************** */
// Shared memory management

int SensorFusion::_setup_shm() {
    // Open SHM file descriptor
    _shm_fd = shm_open(SHM_NAME, O_RDWR, 0666);
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

    // Update read index to latest written message index
    _shm_read_idx = _mav_shm->write_index.load(std::memory_order_acquire);

    return 0;
}

int SensorFusion::_close_shm() {
    munmap((void*) _mav_shm, sizeof(SharedMavlink));
    int result = close(_shm_fd);
    if (result < 0) {
        RCLCPP_ERROR(this->get_logger(), "close() shared memory FD failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

int SensorFusion::_update_imu_data() {
    mavlink_message_t message{};
    mavlink_status_t status{};

    // Get current write and read indexes
    uint64_t shm_write_idx = _mav_shm->write_index.load(std::memory_order_acquire);

    // See if new message are available
    if (_shm_read_idx == shm_write_idx) {
        return 0;
    }

    // Read new message
    SharedMavlink::Slot& read_slot = _mav_shm->slots[_shm_read_idx % QUEUE_SIZE];

    for (uint32_t i = 0; i < read_slot.len; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, read_slot.data[i], &message, &status)) {
            std::cout
                << "Received MAVLink message: "
                << "msgid=" << message.msgid
                << " sysid="
                << static_cast<int>(message.sysid)
                << " compid="
                << static_cast<int>(message.compid)
                << '\n';

            if (message.msgid == MAVLINK_MSG_ID_RAW_IMU) {
                mavlink_raw_imu_t raw_imu{};

                mavlink_msg_raw_imu_decode(
                    &message,
                    &raw_imu);

                std::cout
                    << "  IMU ID = " << static_cast<double>(raw_imu.id) << "\n"
                    << "  time_usec = " << static_cast<double>(raw_imu.time_usec) << "\n"
                    << "  xacc = " << static_cast<double>(raw_imu.xacc) << "\n"
                    << "  yacc = " << static_cast<double>(raw_imu.yacc) << "\n"
                    << "  zacc = " << static_cast<double>(raw_imu.zacc) << "\n"
                    << "  xgyro = " << static_cast<double>(raw_imu.xgyro) << "\n"
                    << "  ygyro = " << static_cast<double>(raw_imu.ygyro) << "\n"
                    << "  zgyro = " << static_cast<double>(raw_imu.zgyro) << "\n"
                    << "  xmag = " << static_cast<double>(raw_imu.xmag) << "\n"
                    << "  ymag = " << static_cast<double>(raw_imu.ymag) << "\n"
                    << "  zmag = " << static_cast<double>(raw_imu.zmag) << "\n"
                    << "  temperature = " << static_cast<double>(raw_imu.temperature)
                    << std::endl;
            }
        }
    }

    // Iterate read pointer
    _shm_read_idx++;

    return 0;
}