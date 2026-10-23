#ifndef BNO086_H
#define BNO086_H

#include "sh2.h"
#include "sh2_SensorValue.h"

/* Call only from the main loop; SH-2 is not reentrant. */
int bno086_init(void);
void bno086_service(void);
typedef enum {
    BNO086_NOT_STARTED = 0,
    BNO086_OPENING = 1,
    BNO086_SETTING_CALLBACK = 2,
    BNO086_READING_PRODUCT_IDS = 3,
    BNO086_CONFIGURING_REPORTS = 4,
    BNO086_RUNNING = 5
} bno086_InitStage;
/* On failure, stage remains at the operation that failed. */
extern volatile bno086_InitStage bno086_init_stage;
extern volatile unsigned bno086_init_done;
extern volatile int bno086_status;
extern volatile uint32_t bno086_report_count;
extern sh2_ProductIds_t bno086_product_ids;
extern sh2_SensorValue_t bno086_latest;

#endif
