#include <atomic>
#include <cstdint>
#include <fcntl.h>


static constexpr const char* SHM_NAME = "/mavlink_shm_sensor_fusion";
static constexpr size_t QUEUE_SIZE = 64;
static constexpr size_t MAX_FRAME_SIZE = 280;

/**
 * @brief A struct containing the shared memory buffer
 *      This struct is shared between two processes
 */
struct SharedMavlink {
    std::atomic<uint64_t> write_index;

    struct Slot {
        uint32_t len;
        uint8_t data[MAX_FRAME_SIZE];
    };

    Slot slots[QUEUE_SIZE];
};