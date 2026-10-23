"""Host-side transport tests: python3 tests/test_bno086_hal.py (requires cc)."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
STUB = r'''
#ifndef MOCK_MAIN_H
#define MOCK_MAIN_H
#include <stdint.h>
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
typedef enum {HAL_OK, HAL_ERROR} HAL_StatusTypeDef;
typedef int SPI_HandleTypeDef;
typedef struct {uint32_t CNT, CR1, PSC, ARR, EGR;} Timer;
typedef struct {uint32_t CFGR;} Clock;
extern Timer timer;
extern Clock clock_regs;
#define TIM5 (&timer)
#define RCC (&clock_regs)
#define RCC_CFGR_PPRE1 0x1c00
#define TIM_EGR_UG 1
#define TIM_CR1_CEN 1
#define SPI1_CS1_GPIO_Port 1
#define SPI1_CS1_Pin 4
#define BNO086_WAKE_GPIO_Port 1
#define BNO086_WAKE_Pin 12
#define BNO086_Interrupt_GPIO_Port 1
#define BNO086_Interrupt_Pin 10
#define EXTI15_10_IRQn 40
#define __HAL_RCC_TIM5_CLK_ENABLE() ((void)0)
#define __HAL_GPIO_EXTI_CLEAR_IT(pin) ((void)0)
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin);
void HAL_GPIO_WritePin(int port, int pin, GPIO_PinState state);
GPIO_PinState HAL_GPIO_ReadPin(int port, int pin);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, unsigned, unsigned);
uint32_t HAL_RCC_GetPCLK1Freq(void);
void HAL_NVIC_SetPriority(int irq, int p, int sub);
void HAL_NVIC_EnableIRQ(int irq);
void HAL_NVIC_DisableIRQ(int irq);
void HAL_Delay(unsigned ms);
#endif
'''
TEST = r'''
#include <assert.h>
#include <stdio.h>
#include "bno086_hal.c"
Timer timer;
Clock clock_regs;
SPI_HandleTypeDef hspi1;
static GPIO_PinState chip_select = GPIO_PIN_SET, wake_state = GPIO_PIN_SET;
static unsigned rx_size, offset, clocks;
static int ready = 1, fail;
static uint8_t sent[1024];
void HAL_GPIO_EXTI_IRQHandler(uint16_t pin) {HAL_GPIO_EXTI_Callback(pin);}
void HAL_GPIO_WritePin(int port, int pin, GPIO_PinState state) {
    (void)port;
    if (pin == 4) {
        chip_select = state;
        if (state == GPIO_PIN_RESET) offset = clocks = 0;
    } else if (pin == 12) wake_state = state;
}
GPIO_PinState HAL_GPIO_ReadPin(int port, int pin) {
    (void)port; (void)pin; timer.CNT += 1000;
    return ready ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *txbuf,
    uint8_t *rxbuf, unsigned size, unsigned timeout) {
    (void)spi; (void)timeout;
    assert(chip_select == GPIO_PIN_RESET);
    if (fail) return HAL_ERROR;
    for (unsigned i=0; i<size; i++,offset++) {
        sent[offset] = txbuf[i];
        rxbuf[i] = offset == 0 ? rx_size & 255 : offset == 1 ? rx_size >> 8 : offset & 255;
    }
    clocks += size;
    return HAL_OK;
}
uint32_t HAL_RCC_GetPCLK1Freq(void) {return 45000000;}
void HAL_NVIC_SetPriority(int irq,int p,int sub) {(void)irq;(void)p;(void)sub;}
void HAL_NVIC_EnableIRQ(int irq) {(void)irq;}
void HAL_NVIC_DisableIRQ(int irq) {(void)irq;}
void HAL_Delay(unsigned ms) {(void)ms;}
static void setup(unsigned length) {
    opened = 1; pending_len = 0; interrupt_seen = 1; interrupt_time = 1234;
    rx_size = length; ready = 1; fail = 0;
}
int main(void) {
    sh2_Hal_t *hal = bno086_get_hal();
    uint8_t buffer[1024], packet[] = {8,0,2,0,10,11,12,13};
    uint32_t stamp = 0;
    setup(20);
    assert(hal->read(hal, buffer, sizeof(buffer), &stamp) == 20);
    assert(stamp == 1234 && clocks == 20 && buffer[19] == 19);
    assert(chip_select == GPIO_PIN_SET);
    setup(32);
    assert(hal->write(hal,packet,sizeof(packet)) == 8);
    assert(clocks == 32 && sent[7] == 13 && sent[8] == 0);
    assert(hal->write(hal,packet,sizeof(packet)) == 0); /* preserve pending RX */
    assert(hal->read(hal,buffer,sizeof(buffer),&stamp) == 32);
    setup(5);
    assert(hal->write(hal,packet,sizeof(packet)) == 8 && clocks == 8);
    assert(hal->read(hal,buffer,sizeof(buffer),&stamp) == 5);
    setup(0);
    assert(hal->read(hal,buffer,sizeof(buffer),&stamp) == 0);
    setup(1025);
    assert(hal->read(hal,buffer,sizeof(buffer),&stamp) == SH2_ERR_IO);
    assert(clocks == 4 && chip_select == GPIO_PIN_SET);
    setup(3);
    assert(hal->read(hal,buffer,sizeof(buffer),&stamp) == SH2_ERR_IO);
    setup(20);
    assert(hal->read(hal,buffer,8,&stamp) == SH2_ERR_BAD_PARAM);
    assert(pending_len == 0);
    setup(20); fail = 1;
    assert(hal->read(hal,buffer,sizeof(buffer),&stamp) == SH2_ERR_IO);
    assert(chip_select == GPIO_PIN_SET);
    setup(0); ready = 0;
    assert(hal->write(hal,packet,sizeof(packet)) == SH2_ERR_TIMEOUT);
    assert(wake_state == GPIO_PIN_SET);
    puts("BNO086 SPI transport tests passed");
}
'''
with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    (tmp / 'main.h').write_text(STUB)
    (tmp / 'spi.h').write_text('#include "main.h"\nextern SPI_HandleTypeDef hspi1;\n')
    (tmp / 'test.c').write_text(TEST)
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(tmp), '-I' + str(ROOT / 'Core/Inc'),
                    '-I' + str(ROOT / 'Core/Src'), str(tmp / 'test.c'),
                    '-o', str(tmp / 'test')], check=True)
    subprocess.run([str(tmp / 'test')], check=True)
