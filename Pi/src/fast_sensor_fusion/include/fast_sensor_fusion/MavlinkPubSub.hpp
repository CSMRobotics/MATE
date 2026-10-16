
#ifndef MAVLINK_PUB_SUB_HPP
#define MAVLINK_PUB_SUB_HPP

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <new>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <mavlink/common/mavlink.h>

static constexpr std::size_t QUEUE_SIZE = 64;
static constexpr std::size_t MAX_FRAME_SIZE = 280;

/**
 * Single-producer, single-consumer MAVLink queue in POSIX shared memory.
 *
 * Start the publisher first so it can initialize a newly created object.
 * The shared-memory object must not be unlinked while either endpoint is
 * using it.
 */
class MavlinkPubSub {
public:
    struct SharedMavlink {
        struct Slot {
            std::uint32_t len;
            std::uint8_t data[MAX_FRAME_SIZE];
        };

        // Monotonically increasing sequence counters.
        std::atomic<std::uint64_t> write_index;
        std::atomic<std::uint64_t> read_index;

        Slot slots[QUEUE_SIZE];
    };

    /**
     * @param write_mode true for publisher, false for subscriber
     * @param channel_name POSIX shared-memory name, e.g. "/mavlink_channel"
     */
    MavlinkPubSub(bool write_mode, const std::string& channel_name)
        : _write_mode(write_mode)
    {
        if (channel_name.empty() || channel_name[0] != '/') {
            throw std::invalid_argument(
                "Shared-memory name must start with '/'");
        }

        bool created = false;

        if (_write_mode) {
            _fd = shm_open(
                channel_name.c_str(),
                O_CREAT | O_EXCL | O_RDWR,
                0660);

            if (_fd >= 0) {
                created = true;
            } else if (errno == EEXIST) {
                _fd = shm_open(channel_name.c_str(), O_RDWR, 0660);
            }
        } else {
            _fd = shm_open(channel_name.c_str(), O_RDWR, 0660);
        }

        if (_fd < 0) {
            throw std::system_error(
                errno, std::generic_category(), "shm_open");
        }

        try {
            if (created) {
                if (ftruncate(
                        _fd,
                        static_cast<off_t>(sizeof(SharedMavlink))) < 0) {
                    throw std::system_error(
                        errno, std::generic_category(), "ftruncate");
                }
            }

            struct stat st {};
            if (fstat(_fd, &st) < 0) {
                throw std::system_error(
                    errno, std::generic_category(), "fstat");
            }

            if (st.st_size < static_cast<off_t>(sizeof(SharedMavlink))) {
                throw std::runtime_error(
                    "Shared-memory object has an invalid size");
            }

            void* addr = mmap(
                nullptr,
                sizeof(SharedMavlink),
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                _fd,
                0);

            if (addr == MAP_FAILED) {
                throw std::system_error(
                    errno, std::generic_category(), "mmap");
            }

            _shm = static_cast<SharedMavlink*>(addr);

            // Only the process that created the object initializes it.
            if (created) {
                ::new (static_cast<void*>(_shm)) SharedMavlink{};

                _shm->read_index.store(0, std::memory_order_relaxed);
                _shm->write_index.store(0, std::memory_order_relaxed);
            } else if (!_shm->write_index.is_lock_free() ||
                       !_shm->read_index.is_lock_free()) {
                throw std::runtime_error(
                    "Shared-memory atomics must be lock-free");
            }

            if (_write_mode &&
                (!_shm->write_index.is_lock_free() ||
                 !_shm->read_index.is_lock_free())) {
                throw std::runtime_error(
                    "Shared-memory atomics must be lock-free");
            }

            // Begin reading at the current queue position.
            // Existing queued messages are not replayed to a new subscriber.
            if (!_write_mode) {
                _shm->read_index.store(
                    _shm->write_index.load(std::memory_order_acquire),
                    std::memory_order_release);
            }
        } catch (...) {
            cleanup();
            throw;
        }
    }

    ~MavlinkPubSub() {
        cleanup();
    }

    MavlinkPubSub(const MavlinkPubSub&) = delete;
    MavlinkPubSub& operator=(const MavlinkPubSub&) = delete;
    MavlinkPubSub(MavlinkPubSub&&) = delete;
    MavlinkPubSub& operator=(MavlinkPubSub&&) = delete;

    /**
     * Write one serialized MAVLink frame.
     *
     * @return 0 on success, -1 if the queue is full
     * @throws std::logic_error if called on a subscriber
     */
    int write(const mavlink_message_t& msg) {
        if (!_write_mode) {
            throw std::logic_error(
                "Cannot write using a subscriber");
        }

        std::uint8_t frame[MAX_FRAME_SIZE];

        const std::uint16_t frame_len =
            mavlink_msg_to_send_buffer(frame, &msg);

        if (frame_len == 0 || frame_len > MAX_FRAME_SIZE) {
            throw std::runtime_error("Invalid MAVLink frame length");
        }

        const std::uint64_t write =
            _shm->write_index.load(std::memory_order_relaxed);

        const std::uint64_t read =
            _shm->read_index.load(std::memory_order_acquire);

        // Do not overwrite messages the consumer has not read.
        if (write - read >= QUEUE_SIZE) {
            return -1;
        }

        auto& slot = _shm->slots[write % QUEUE_SIZE];

        std::memcpy(slot.data, frame, frame_len);
        slot.len = frame_len;

        // Publish the completed slot.
        _shm->write_index.store(
            write + 1,
            std::memory_order_release);

        return 0;
    }

    /**
     * Consume one queued frame and parse it as MAVLink.
     *
     * @return 1 if a complete message was parsed
     *         0 if a frame was consumed but no complete message was parsed
     *        -1 if the queue is empty
     * @throws std::logic_error if called on a publisher
     */
    int read(mavlink_message_t& msg, mavlink_status_t& status) {
        if (_write_mode) {
            throw std::logic_error(
                "Cannot read using a publisher");
        }

        const std::uint64_t read =
            _shm->read_index.load(std::memory_order_relaxed);

        const std::uint64_t write =
            _shm->write_index.load(std::memory_order_acquire);

        if (read == write) {
            return -1;
        }

        auto& slot = _shm->slots[read % QUEUE_SIZE];

        // Validate before accessing the frame buffer.
        const std::uint32_t len = slot.len;

        if (len == 0 || len > MAX_FRAME_SIZE) {
            // Discard the malformed slot to avoid blocking the queue.
            _shm->read_index.store(
                read + 1,
                std::memory_order_release);
            return 0;
        }

        bool parsed = false;
        mavlink_message_t parsed_msg {};
        mavlink_status_t parsed_status {};

        for (std::uint32_t i = 0; i < len; ++i) {
            if (mavlink_parse_char(
                    MAVLINK_COMM_0,
                    slot.data[i],
                    &parsed_msg,
                    &parsed_status)) {
                parsed = true;
            }
        }

        // Release the slot so the publisher can reuse it.
        _shm->read_index.store(
            read + 1,
            std::memory_order_release);

        if (!parsed) {
            return 0;
        }

        msg = parsed_msg;
        status = parsed_status;

        std::cout
            << "Received MAVLink message: msgid=" << msg.msgid
            << " sysid=" << static_cast<int>(msg.sysid)
            << " compid=" << static_cast<int>(msg.compid)
            << '\n';

        return 1;
    }

private:
    void cleanup() noexcept {
        if (_shm != nullptr) {
            munmap(_shm, sizeof(SharedMavlink));
            _shm = nullptr;
        }

        if (_fd >= 0) {
            close(_fd);
            _fd = -1;
        }
    }

    bool _write_mode = false;
    int _fd = -1;
    SharedMavlink* _shm = nullptr;
};

#endif // MAVLINK_PUB_SUB_HPP
