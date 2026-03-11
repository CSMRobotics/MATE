/*
 * dshot.h
 *
 *  Created on: 2021. 1. 27.
 *      Author: mokhwasomssi
 */


#ifndef __DSHOT_H__
#define __DSHOT_H__


#include "tim.h"    	// header from stm32cubemx code generate
#include <stdbool.h>	
#include <math.h>		// lrintf


/* User Configuration */
// Timer Clock
#define TIMER_CLOCK				90000000	// 90MHz

// MOTOR 1 (PA0) - TIM2 Channel 1, DMA1 Stream 5
#define MOTOR_1_TIM             (&htim2)
#define MOTOR_1_TIM_CHANNEL     TIM_CHANNEL_1
#define MOTOR1_DMA_ID TIM_DMA_ID_CC1
#define MOTOR1_DMA TIM_DMA_CC1
#define MOTOR1_PULSE_REGISTER CCR1

// MOTOR 2 (PA1) - TIM2 Channel 2, DMA1 Stream 6
#define MOTOR_2_TIM             (&htim2)
#define MOTOR_2_TIM_CHANNEL     TIM_CHANNEL_2
#define MOTOR2_DMA_ID TIM_DMA_ID_CC2
#define MOTOR2_DMA TIM_DMA_CC2
#define MOTOR2_PULSE_REGISTER CCR2

// MOTOR 3 (PB10) - TIM2 Channel 3, DMA1 Stream 1
#define MOTOR_3_TIM             (&htim2)
#define MOTOR_3_TIM_CHANNEL     TIM_CHANNEL_3
#define MOTOR3_DMA_ID TIM_DMA_ID_CC3
#define MOTOR3_DMA TIM_DMA_CC3
#define MOTOR3_PULSE_REGISTER CCR3

// MOTOR 4 (PB4) - TIM3 Channel 1, DMA1 Stream 4
#define MOTOR_4_TIM             (&htim3)
#define MOTOR_4_TIM_CHANNEL     TIM_CHANNEL_1
#define MOTOR4_DMA_ID TIM_DMA_ID_CC1
#define MOTOR4_DMA TIM_DMA_CC1
#define MOTOR4_PULSE_REGISTER CCR1

// MOTOR 5 (PB1) - TIM3 Channel 4, DMA1 Stream 2
#define MOTOR_5_TIM             (&htim3)
#define MOTOR_5_TIM_CHANNEL     TIM_CHANNEL_4
#define MOTOR5_DMA_ID TIM_DMA_ID_CC4
#define MOTOR5_DMA TIM_DMA_CC4
#define MOTOR5_PULSE_REGISTER CCR4

// MOTOR 6 (PB6) - TIM4 Channel 1, DMA1 Stream 0
#define MOTOR_6_TIM             (&htim4)
#define MOTOR_6_TIM_CHANNEL     TIM_CHANNEL_1
#define MOTOR6_DMA_ID TIM_DMA_ID_CC1
#define MOTOR6_DMA TIM_DMA_CC1
#define MOTOR6_PULSE_REGISTER CCR1

// MOTOR 7 (PB7) - TIM4 Channel 2, DMA1 Stream 3
#define MOTOR_7_TIM             (&htim4)
#define MOTOR_7_TIM_CHANNEL     TIM_CHANNEL_2
#define MOTOR7_DMA_ID TIM_DMA_ID_CC2
#define MOTOR7_DMA TIM_DMA_CC2
#define MOTOR7_PULSE_REGISTER CCR2

// MOTOR 8 (PB8) - TIM4 Channel 3, DMA1 Stream 7
#define MOTOR_8_TIM             (&htim4)
#define MOTOR_8_TIM_CHANNEL     TIM_CHANNEL_3
#define MOTOR8_DMA_ID TIM_DMA_ID_CC3
#define MOTOR8_DMA TIM_DMA_CC3
#define MOTOR8_PULSE_REGISTER CCR3


/* Definition */
#define MHZ_TO_HZ(x) 			((x) * 1000000)

#define DSHOT600_HZ     		MHZ_TO_HZ(12)
#define DSHOT300_HZ     		MHZ_TO_HZ(6)
#define DSHOT150_HZ     		MHZ_TO_HZ(3)

#define MOTOR_BIT_0            	7
#define MOTOR_BIT_1            	14
#define MOTOR_BITLENGTH        	20

#define DSHOT_FRAME_SIZE       	16
#define DSHOT_DMA_BUFFER_SIZE   18 /* resolution + frame reset (2us) */

#define DSHOT_MIN_THROTTLE      48
#define DSHOT_MAX_THROTTLE     	2047
#define DSHOT_RANGE 			(DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)


/* Enumeration */
typedef enum
{
    DSHOT150,
    DSHOT300,
    DSHOT600
} dshot_type_e;


/* Functions */
void dshot_init(dshot_type_e dshot_type);
void dshot_write(uint16_t* motor_value);


#endif /* __DSHOT_H__ */
