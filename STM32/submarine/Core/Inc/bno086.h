#ifndef BNO086_H
#define BNO086_H

#include "sh2.h"
#include "sh2_SensorValue.h"

/* Call only from the main loop; SH-2 is not reentrant. */
int bno086_init(void);
void bno086_service(void);
extern volatile int bno086_status;
extern volatile uint32_t bno086_report_count;
extern sh2_ProductIds_t bno086_product_ids;
extern sh2_SensorValue_t bno086_latest;

#endif
