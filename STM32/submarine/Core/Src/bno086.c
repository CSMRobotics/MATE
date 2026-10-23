#include "bno086.h"
#include "bno086_hal.h"
#include "sh2_err.h"
#include "main.h"
#include <stdbool.h>

volatile int bno086_status = SH2_ERR;
volatile uint32_t bno086_report_count;
sh2_ProductIds_t bno086_product_ids;
sh2_SensorValue_t bno086_latest;
static bool reset_pending;
static bool active;
static uint32_t retry_at;

static void event_callback(void *cookie, sh2_AsyncEvent_t *event)
{
    (void)cookie;
    if (event->eventId == SH2_RESET) reset_pending = true;
}

static void sensor_callback(void *cookie, sh2_SensorEvent_t *event)
{
    (void)cookie;
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) == SH2_OK &&
        value.sensorId == SH2_ROTATION_VECTOR) {
        bno086_latest = value;
        bno086_report_count++;
    }
}

static int configure_reports(void)
{
    sh2_SensorConfig_t config = {0};
    config.reportInterval_us = 20000; /* 50 Hz */
    return sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config);
}

int bno086_init(void)
{
    active = false;
    reset_pending = false;
    bno086_report_count = 0;
    bno086_get_hal()->close(bno086_get_hal());
    int rc = sh2_open(bno086_get_hal(), event_callback, 0);
    if (rc == SH2_OK) rc = sh2_setSensorCallback(sensor_callback, 0);
    if (rc == SH2_OK) rc = sh2_getProdIds(&bno086_product_ids);
    if (rc == SH2_OK) {
        reset_pending = false;
        rc = configure_reports();
    }
    bno086_status = rc;
    active = rc == SH2_OK;
    if (!active) {
        sh2_close();
        bno086_get_hal()->close(bno086_get_hal());
    }
    return rc;
}

void bno086_service(void)
{
    if (!active) return;
    sh2_service();
    /* Defer API calls out of callbacks; retry configuration after a reset. */
    if (reset_pending && (int32_t)(HAL_GetTick() - retry_at) >= 0) {
        reset_pending = false;
        bno086_status = configure_reports();
        if (bno086_status != SH2_OK) {
            reset_pending = true;
            retry_at = HAL_GetTick() + 1000;
        }
    }
}
