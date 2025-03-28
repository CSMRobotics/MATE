#include "rov_bno085/bno08x_driver.hpp"

#include <chrono>
#include <thread>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <stdexcept>

#include "jetgpio.h"

extern "C" {
#include "sh2.h"
#include "sh2_err.h"
}

#define SPIDEVICE "/dev/spidev1.0"

// HAL garbage
static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

// TODO: implement
// static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
// static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
// static void i2chal_close(sh2_Hal_t *self);
// static int i2chal_open(sh2_Hal_t *self);

// TODO: implement
// static int uarthal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
// static int uarthal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
// static void uarthal_close(sh2_Hal_t *self);
// static int uarthal_open(sh2_Hal_t *self);

static bool spihal_wait_for_int(void);
static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static void spihal_close(sh2_Hal_t *self);
static int spihal_open(sh2_Hal_t *self);
static uint8_t spi_mode = SPI_MODE_3;
static uint8_t spi_bits_per_word = 8;
static uint32_t spi_speed = 1e6; // 1 MHz
static uint8_t spi_lsb_first = 0;
static uint16_t spi_delay = 0;
static std::chrono::time_point<std::chrono::high_resolution_clock> spi_start;

static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);
static void hal_hardwareReset(void);

// shared stuff
static int _fd;
static unsigned int _reset_pin;
static unsigned int _interrupt_pin;

BNO08X::BNO08X(uint8_t reset_pin, uint8_t interrupt_pin) {
    int ret;
    ret = gpioInitialise();

    if (ret < 0) {
        printf("Jetgpio initialization failed with errco %d\n", ret);
        throw std::logic_error("Jetgpio failed to initalize");
    }
    
    // setup reset pin as output
    this->reset_pin = reset_pin;
    _reset_pin = this->reset_pin;
    ret = gpioSetMode(reset_pin, JET_OUTPUT);

    // setup "interrupt" pin as input
    this->interrupt_pin = interrupt_pin;
    _interrupt_pin = this->interrupt_pin;
    ret = gpioSetMode(interrupt_pin, JET_INPUT);
}

BNO08X::~BNO08X() {
    gpioTerminate();
}

bool BNO08X::makeSPI(int32_t sensor_id) {
    // if file descriptor is in use, we need to close it
    if (fd != -1) {
        close(fd);
        // TODO: if uart and i2c have local fds, invalidate them now
    }

    // open the spi device
    fd = open(SPIDEVICE, O_RDWR);
    if (fd < 0) {
        perror("Unable to open SPI device");
        return false;
    }
    _fd = fd;

    // set spi mode
    if (ioctl(fd, SPI_IOC_WR_MODE, &spi_mode) == -1) {
        perror("Unable to set SPI mode");
        return false;
    }

    // set bits per word
    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits_per_word) == -1) {
        perror("Unable to set bits per word");
        return false;
    }

    // set max speed
    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed) == -1) {
        perror("Unable to set max speed");
        return false;
    }

    // set bit order
    if (ioctl(fd, SPI_IOC_WR_LSB_FIRST, &spi_lsb_first) == -1) {
        perror("Unable to set bitorder to msb first");
        return false;
    }

    _HAL.open = spihal_open;
    _HAL.close = spihal_close;
    _HAL.read = spihal_read;
    _HAL.write = spihal_write;
    _HAL.getTimeUs = hal_getTimeUs;

    spi_start = std::chrono::high_resolution_clock::now();

    return true;
}

bool BNO08X::_init(int32_t sensor_id) {
    int status;

    hardwareReset();

    // open SH2
    status = sh2_open(&_HAL, hal_callback, NULL);
    if (status != SH2_OK) {
        return false;
    }

    // check connection
    memset(&product_ids, 0, sizeof(product_ids));
    status = sh2_getProdIds(&product_ids);
    if (status != SH2_OK) {
        return false;
    }

    // register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

    return true;
}

void BNO08X::hardwareReset(void) {
    hal_hardwareReset();
}

bool BNO08X::wasReset(void) {
    bool x = _reset_occurred;
    _reset_occurred = false;
    return x;
}

bool BNO08X::enableReport(sh2_SensorId_t sensor_id, uint32_t interval_us) {
    static sh2_SensorConfig_t config;
    
    // sane defaults
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    config.reportInterval_us = interval_us;
    int status = sh2_setSensorConfig(sensor_id, &config);

    if (status != SH2_OK) {
        return false;
    }
    return true;
}

bool BNO08X::getSensorEvent(sh2_SensorValue_t *value) {
    _sensor_value = value;
    value->timestamp = 0;

    sh2_service();

    // check if there were new events
    if (value->timestamp == 0 && value->sensorId != SH2_GYRO_INTEGRATED_RV)  {
        return false;
    }

    return true;
}


static int spi_transfer(int fd, uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    struct spi_ioc_transfer tr{};
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf = reinterpret_cast<unsigned long>(tx_buf);
    tr.rx_buf = reinterpret_cast<unsigned long>(rx_buf);
    tr.len = len;
    tr.speed_hz = spi_speed;
    tr.delay_usecs = spi_delay;
    tr.bits_per_word = spi_bits_per_word;

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("SPI_IOC_MESSAGE failed");
        return -1;
    }
    
    return 0;
}

static bool spihal_wait_for_int(void) {
    // this is actually really stupid
    // does chip interrupt within 0.5 seconds?
    for (int i = 0; i < 500; i++) {
        if (gpioRead(_interrupt_pin)) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // if chip does not interrupt, reset it
    hal_hardwareReset();
    
    return false;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    static uint8_t rx[4] = {0};
    if (!spihal_wait_for_int()) {
        return 0;
    }

    memset(rx, 0, sizeof(rx));
    int ret = spi_transfer(_fd, pBuffer, rx, len);
    
    // adafruit doesnt do this?
    // if (ret < 0) {
    //     return 0;
    // }

    return len;
}

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
    static uint16_t packet_size = 0;
    static uint8_t tx[4] = {0};
    if (!spihal_wait_for_int()) {
        return 0;
    }

    // read spi
    memset(tx, 0, sizeof(tx));
    int ret = spi_transfer(_fd, tx, pBuffer, 4);
    if (ret < 0) {
        return 0;
    }
    
    // determine amount to read
    packet_size = static_cast<uint16_t>(pBuffer[0]) | (static_cast<uint16_t>(pBuffer[1]) << 8);
    // unset continue bit
    packet_size &= 0x8000;

    if (packet_size > len) {
        return 0;
    }
    
    if (!spihal_wait_for_int()) {
        return 0;
    }
    
    // read packet_size bytes into pBuffer
    memset(tx, 0, sizeof(tx));
    ret = spi_transfer(_fd, tx, pBuffer, packet_size);
    if (ret < 0) {
        return 0;
    }
    
    // return how many bytes were read
    return packet_size;
}

static void spihal_close(sh2_Hal_t *self) {
    // does nothing
}

static int spihal_open(sh2_Hal_t *self) {
    spihal_wait_for_int();

    return 0;
}

uint32_t hal_getTimeUs(sh2_Hal_t *self) {
    auto tp = std::chrono::high_resolution_clock::now();
    // ~1 hour 71 minutes before it overflows....
    int64_t time = std::chrono::duration_cast<std::chrono::microseconds>(tp - spi_start).count();
    // handle 32bit overflow by wrapping back to 0
    if (time > UINT32_MAX) {
        time = time - UINT32_MAX;
    }

    return static_cast<uint32_t>(time);
}

void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    if (pEvent->eventId == SH2_RESET) {
        _reset_occurred = true;
    }
}

void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent) {
    int ret;
    ret = sh2_decodeSensorEvent(_sensor_value, pEvent);

    if (ret != SH2_OK) {
        printf("BNO08x - Error decoding sensor event\n");
        _sensor_value->timestamp = 0;
        return;
    }
}

void hal_hardwareReset(void) {
    gpioWrite(_reset_pin, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpioWrite(_reset_pin, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    gpioWrite(_reset_pin, 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}