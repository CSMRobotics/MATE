#ifndef BNO086HAL_H
#define BNO086HAL_H

#include "sh2_hal.h"

static int bno_open(sh2_Hal_t *self);
static void bno_close(sh2_Hal_t *self);
static int bno_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static int bno_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

#endif
