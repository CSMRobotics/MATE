#ifndef UART_MAVLINK_H
#define UART_MAVLINK_H

#include "mavlink/rov_interfaces/mavlink.h"
#include "main.h"


#include <string>

extern UART_HandleTypeDef huart1; // defined in main.c so i think you have to do extern
extern UART_HandleTypeDef huart2;

//********************************************************************************************
// Transmit

/**
 * @brief Log a message to Pi (Flight Computer)
 *
 * @param log_msg: string to log, MAX SIZE OF 50 CHARS
 * @param servirity: serverity of this message
 */
void mavlink_log(const std::string& log_msg, MAV_SEVERITY severity);

//********************************************************************************************
// Receive
extern volatile uint8_t sub_controls_received;
extern mavlink_sub_controls_t latest_sub_controls;

/**
 * @brief Initializes the interrupt driven receive channel
 */
void init_uart_mavlink_rx();

#endif
