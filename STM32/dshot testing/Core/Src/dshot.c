/*
 * dshot.h
 *
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 *
 */


#include "dshot.h"


/* Variables */ // added more motors
static uint32_t motor1_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor2_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor3_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor4_dmabuffer[DSHOT_DMA_BUFFER_SIZE];

static uint32_t motor5_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor6_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor7_dmabuffer[DSHOT_DMA_BUFFER_SIZE];
static uint32_t motor8_dmabuffer[DSHOT_DMA_BUFFER_SIZE];





/* Static functions */
// dshot init
static uint32_t dshot_choose_type(dshot_type_e dshot_type);
static void dshot_set_timer(dshot_type_e dshot_type);
static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma);
static void dshot_put_tc_callback_function();
static void dshot_start_pwm();

// dshot write
static uint16_t dshot_prepare_packet(uint16_t value);
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value);
static void dshot_prepare_dmabuffer_all();
static void dshot_dma_start();
static void dshot_enable_dma_request();


/* Functions */
void dshot_init(dshot_type_e dshot_type)
{
	dshot_set_timer(dshot_type);
	dshot_put_tc_callback_function();
	dshot_start_pwm();
}

void dshot_write(uint16_t* motor_value)
{
	dshot_prepare_dmabuffer_all(motor_value);
	dshot_dma_start();
	dshot_enable_dma_request();
}


/* Static functions */
static uint32_t dshot_choose_type(dshot_type_e dshot_type)
{
	switch (dshot_type)
	{
		case(DSHOT600):
				return DSHOT600_HZ;

		case(DSHOT300):
				return DSHOT300_HZ;

		default:
		case(DSHOT150):
				return DSHOT150_HZ;
	}
}

// added other motors
static void dshot_set_timer(dshot_type_e dshot_type)
{
	uint16_t dshot_prescaler;
	uint32_t timer_clock = TIMER_CLOCK; // all timer clock is same as SystemCoreClock in stm32f411

	// Calculate prescaler by dshot type
	dshot_prescaler = lrintf((float) timer_clock / dshot_choose_type(dshot_type) + 0.01f) - 1;

	// motor1
	__HAL_TIM_SET_PRESCALER(MOTOR_1_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_1_TIM, MOTOR_BITLENGTH);

	// motor2
	__HAL_TIM_SET_PRESCALER(MOTOR_2_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_2_TIM, MOTOR_BITLENGTH);

	// motor3
	__HAL_TIM_SET_PRESCALER(MOTOR_3_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_3_TIM, MOTOR_BITLENGTH);

	// motor4
	__HAL_TIM_SET_PRESCALER(MOTOR_4_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_4_TIM, MOTOR_BITLENGTH);

	// motor5
	__HAL_TIM_SET_PRESCALER(MOTOR_5_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_5_TIM, MOTOR_BITLENGTH);

	// motor6
	__HAL_TIM_SET_PRESCALER(MOTOR_6_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_6_TIM, MOTOR_BITLENGTH);

	// motor7
	__HAL_TIM_SET_PRESCALER(MOTOR_7_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_7_TIM, MOTOR_BITLENGTH);

	// motor8
	__HAL_TIM_SET_PRESCALER(MOTOR_8_TIM, dshot_prescaler);
	__HAL_TIM_SET_AUTORELOAD(MOTOR_8_TIM, MOTOR_BITLENGTH);
}

// __HAL_TIM_DISABLE_DMA is needed to eliminate the delay between different dshot signals
// I don't know why :(
static void dshot_dma_tc_callback(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	}
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
	{
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
	}
}

static void dshot_put_tc_callback_function() // added more motors and changed dma id to defined in the header
{
	// TIM_DMA_ID_CCx depends on timer channel
	MOTOR_1_TIM->hdma[MOTOR1_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_2_TIM->hdma[MOTOR2_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_3_TIM->hdma[MOTOR3_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_4_TIM->hdma[MOTOR4_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;

	MOTOR_5_TIM->hdma[MOTOR5_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_6_TIM->hdma[MOTOR6_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_7_TIM->hdma[MOTOR7_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;
	MOTOR_8_TIM->hdma[MOTOR8_DMA_ID]->XferCpltCallback = dshot_dma_tc_callback;
}

static void dshot_start_pwm() // added more motors
{
	// Start the timer channel now.
    // Enabling/disabling DMA request can restart a new cycle without PWM start/stop.
  	HAL_TIM_PWM_Start(MOTOR_1_TIM, MOTOR_1_TIM_CHANNEL);
  	HAL_TIM_PWM_Start(MOTOR_2_TIM, MOTOR_2_TIM_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_3_TIM, MOTOR_3_TIM_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_4_TIM, MOTOR_4_TIM_CHANNEL);

	HAL_TIM_PWM_Start(MOTOR_5_TIM, MOTOR_5_TIM_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_6_TIM, MOTOR_6_TIM_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_7_TIM, MOTOR_7_TIM_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR_8_TIM, MOTOR_8_TIM_CHANNEL);

}

static uint16_t dshot_prepare_packet(uint16_t value)
{
	uint16_t packet;
	bool dshot_telemetry = false;

	packet = (value << 1) | (dshot_telemetry ? 1 : 0);

	// compute checksum
	unsigned csum = 0;
	unsigned csum_data = packet;

	for(int i = 0; i < 3; i++)
	{
        csum ^=  csum_data; // xor data by nibbles
        csum_data >>= 4;
	}

	csum &= 0xf;
	packet = (packet << 4) | csum;

	return packet;
}

// Convert 16 bits packet to 16 pwm signal
static void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value)
{
	uint16_t packet;
	packet = dshot_prepare_packet(value);

	for(int i = 0; i < 16; i++)
	{
		motor_dmabuffer[i] = (packet & 0x8000) ? MOTOR_BIT_1 : MOTOR_BIT_0;
		packet <<= 1;
	}

	motor_dmabuffer[16] = 0;
	motor_dmabuffer[17] = 0;
}

static void dshot_prepare_dmabuffer_all(uint16_t *motor_value) // added more motors
{
	dshot_prepare_dmabuffer(motor1_dmabuffer, motor_value[0]);
	dshot_prepare_dmabuffer(motor2_dmabuffer, motor_value[1]);
	dshot_prepare_dmabuffer(motor3_dmabuffer, motor_value[2]);
	dshot_prepare_dmabuffer(motor4_dmabuffer, motor_value[3]);

	dshot_prepare_dmabuffer(motor5_dmabuffer, motor_value[4]);
	dshot_prepare_dmabuffer(motor6_dmabuffer, motor_value[5]);
	dshot_prepare_dmabuffer(motor7_dmabuffer, motor_value[6]);
	dshot_prepare_dmabuffer(motor8_dmabuffer, motor_value[7]);

}

static void dshot_dma_start() // changed dma id and timer counter register to a define in header and added more motors
{
	HAL_DMA_Start_IT(MOTOR_1_TIM->hdma[MOTOR1_DMA_ID], (uint32_t) motor1_dmabuffer, (uint32_t) &MOTOR_1_TIM->Instance->MOTOR1_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(MOTOR_2_TIM->hdma[MOTOR2_DMA_ID], (uint32_t) motor2_dmabuffer, (uint32_t) &MOTOR_2_TIM->Instance->MOTOR2_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(MOTOR_3_TIM->hdma[MOTOR3_DMA_ID], (uint32_t) motor3_dmabuffer, (uint32_t) &MOTOR_3_TIM->Instance->MOTOR3_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(MOTOR_4_TIM->hdma[MOTOR4_DMA_ID], (uint32_t) motor4_dmabuffer, (uint32_t) &MOTOR_4_TIM->Instance->MOTOR4_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);

	HAL_DMA_Start_IT(MOTOR_5_TIM->hdma[MOTOR5_DMA_ID], (uint32_t) motor5_dmabuffer, (uint32_t) &MOTOR_5_TIM->Instance->MOTOR5_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(MOTOR_6_TIM->hdma[MOTOR6_DMA_ID], (uint32_t) motor6_dmabuffer, (uint32_t) &MOTOR_6_TIM->Instance->MOTOR6_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(MOTOR_7_TIM->hdma[MOTOR7_DMA_ID], (uint32_t) motor7_dmabuffer, (uint32_t) &MOTOR_7_TIM->Instance->MOTOR7_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);
	HAL_DMA_Start_IT(MOTOR_8_TIM->hdma[MOTOR8_DMA_ID], (uint32_t) motor8_dmabuffer, (uint32_t) &MOTOR_8_TIM->Instance->MOTOR8_PULSE_REGISTER, DSHOT_DMA_BUFFER_SIZE);
}

static void dshot_enable_dma_request() // motor dma to header thing and more motor
{
	__HAL_TIM_ENABLE_DMA(MOTOR_1_TIM, MOTOR1_DMA);
	__HAL_TIM_ENABLE_DMA(MOTOR_2_TIM, MOTOR2_DMA);
	__HAL_TIM_ENABLE_DMA(MOTOR_3_TIM, MOTOR3_DMA);
	__HAL_TIM_ENABLE_DMA(MOTOR_4_TIM, MOTOR4_DMA);

	__HAL_TIM_ENABLE_DMA(MOTOR_5_TIM, MOTOR5_DMA);
	__HAL_TIM_ENABLE_DMA(MOTOR_6_TIM, MOTOR6_DMA);
	__HAL_TIM_ENABLE_DMA(MOTOR_7_TIM, MOTOR7_DMA);
	__HAL_TIM_ENABLE_DMA(MOTOR_8_TIM, MOTOR8_DMA);
}
