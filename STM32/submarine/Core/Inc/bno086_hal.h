#ifndef BNO086_HAL_H
#define BNO086_HAL_H
#include "sh2_hal.h"
sh2_Hal_t *bno086_get_hal(void);
typedef struct {
    int last_error; /* SH2 error; retained across close for debugging */
    int spi_status; /* STM32 HAL status of last SPI operation */
    unsigned intn_level; /* Last level observed: 0 = ready, 1 = high */
    uint32_t interrupt_count;
    uint32_t transfer_count;
    uint8_t rx_header[4];
} bno086_HalDiagnostics;
extern volatile bno086_HalDiagnostics bno086_hal_diag;
#endif
