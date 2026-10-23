/* Waterboard SPI1 transport. Protocol reference:
 * https://github.com/ceva-dsp/sh2-demo-nucleo/blob/main/app/spi_hal.c
 * Blocking transfers are confined to main context; EXTI only timestamps INTN.
 */
#include "bno086_hal.h"
#include "main.h"
#include "spi.h"
#include "sh2_err.h"
#include <string.h>

static uint8_t rx[SH2_HAL_MAX_TRANSFER_IN];
static uint8_t tx[SH2_HAL_MAX_TRANSFER_IN];
static unsigned pending_len;
static uint32_t pending_time;
static volatile uint32_t interrupt_time;
static volatile unsigned interrupt_seen;
static unsigned opened;
volatile bno086_HalDiagnostics bno086_hal_diag;

static int transport_error(int error)
{
    bno086_hal_diag.last_error = error;
    return error;
}

static GPIO_PinState intn(void)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(BNO086_Interrupt_GPIO_Port,
                                          BNO086_Interrupt_Pin);
    bno086_hal_diag.intn_level = state;
    return state;
}

static uint32_t time_us(sh2_Hal_t *self)
{
    (void)self;
    return TIM5->CNT;
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BNO086_Interrupt_Pin);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin == BNO086_Interrupt_Pin) {
        bno086_hal_diag.interrupt_count++;
        interrupt_time = TIM5->CNT;
        interrupt_seen = 1;
    }
}

static void cs(GPIO_PinState state)
{
    HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, state);
}

static void wake(GPIO_PinState state)
{
    HAL_GPIO_WritePin(BNO086_WAKE_GPIO_Port, BNO086_WAKE_Pin, state);
}

/* Exchange header first, then clock enough bytes for BOTH directions.
 * CS stays asserted throughout. Zero TX length denotes a read.
 */
static int exchange(const uint8_t *data, unsigned size)
{
    memset(tx, 0, sizeof(tx));
    if (size) memcpy(tx, data, size);
    pending_time = interrupt_seen ? interrupt_time : time_us(0);
    interrupt_seen = 0;
    cs(GPIO_PIN_RESET);
    HAL_StatusTypeDef rc = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 4, 20);
    bno086_hal_diag.transfer_count++;
    bno086_hal_diag.spi_status = rc;
    if (rc == HAL_OK) {
        for (unsigned i = 0; i < 4; i++) bno086_hal_diag.rx_header[i] = rx[i];
    }
    wake(GPIO_PIN_SET);
    unsigned received = 0;
    if (rc == HAL_OK) received = (rx[0] | ((unsigned)rx[1] << 8)) & 0x7fff;
    if (received > sizeof(rx) || (received && received < 4)) {
        cs(GPIO_PIN_SET);
        return transport_error(SH2_ERR_IO);
    }
    unsigned total = size > received ? size : received;
    if (rc == HAL_OK && total > 4)
        rc = HAL_SPI_TransmitReceive(&hspi1, tx + 4, rx + 4, total - 4, 20);
    cs(GPIO_PIN_SET);
    bno086_hal_diag.spi_status = rc;
    if (rc != HAL_OK) return transport_error(SH2_ERR_IO);
    pending_len = received > 4 ? received : 0;
    return SH2_OK;
}

static int write_transfer(sh2_Hal_t *self, uint8_t *data, unsigned len)
{
    (void)self;
    if (!opened || !data || len < 4 || len > SH2_HAL_MAX_TRANSFER_OUT)
        return SH2_ERR_BAD_PARAM;
    if (pending_len) return 0;
    wake(GPIO_PIN_RESET);
    uint32_t start = time_us(0);
    while (intn() != GPIO_PIN_RESET) {
        if ((uint32_t)(time_us(0) - start) >= 20000) {
            wake(GPIO_PIN_SET);
            return transport_error(SH2_ERR_TIMEOUT);
        }
    }
    int rc = exchange(data, len);
    return rc == SH2_OK ? (int)len : rc;
}

static int read_transfer(sh2_Hal_t *self, uint8_t *data, unsigned len, uint32_t *stamp)
{
    (void)self;
    if (!opened || !data || !stamp) return SH2_ERR_BAD_PARAM;
    if (!pending_len) {
        if (intn() != GPIO_PIN_RESET) return 0;
        int rc = exchange(0, 0);
        if (rc != SH2_OK) return rc;
    }
    unsigned count = pending_len;
    pending_len = 0;
    if (count > len) return SH2_ERR_BAD_PARAM;
    if (count) {
        memcpy(data, rx, count);
        *stamp = pending_time;
    }
    return (int)count;
}

static int open_transport(sh2_Hal_t *self)
{
    if (opened) return SH2_ERR;
    bno086_hal_diag = (bno086_HalDiagnostics){0};
    /* TIM2/3/4 belong to DShot. Reserve unused 32-bit TIM5 at 1 MHz. */
    __HAL_RCC_TIM5_CLK_ENABLE();
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) timer_clock *= 2;
    TIM5->CR1 = 0;
    TIM5->PSC = timer_clock / 1000000 - 1;
    TIM5->ARR = 0xffffffff;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->CR1 = TIM_CR1_CEN;
    pending_len = 0;
    interrupt_seen = 0;
    cs(GPIO_PIN_SET);
    wake(GPIO_PIN_SET);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    __HAL_GPIO_EXTI_CLEAR_IT(BNO086_Interrupt_Pin);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    opened = 1;
    /* Establish idle-high SCLK before selecting the sensor. */
    uint8_t dummy_tx = 0, dummy_rx;
    HAL_StatusTypeDef spi_rc = HAL_SPI_TransmitReceive(&hspi1, &dummy_tx, &dummy_rx, 1, 20);
    bno086_hal_diag.spi_status = spi_rc;
    if (spi_rc != HAL_OK) return transport_error(SH2_ERR_IO);
    HAL_Delay(300);
    /* RESET is pulled high: use SHTP executable channel reset command.
     * Also covers MCU-only restart while the sensor remains powered.
     */
    uint8_t reset[] = {5, 0, 1, 0, 1};
    int rc = write_transfer(self, reset, sizeof(reset));
    return rc == sizeof(reset) ? SH2_OK : (rc < 0 ? rc : SH2_ERR_IO);
}

static void close_transport(sh2_Hal_t *self)
{
    (void)self;
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    cs(GPIO_PIN_SET);
    wake(GPIO_PIN_SET);
    pending_len = 0;
    opened = 0;
    /* No reset GPIO; leave shared SPI1 and the timestamp timer initialized. */
}

sh2_Hal_t *bno086_get_hal(void)
{
    static sh2_Hal_t hal = {
        .open = open_transport, .close = close_transport,
        .read = read_transfer, .write = write_transfer, .getTimeUs = time_us
    };
    return &hal;
}
