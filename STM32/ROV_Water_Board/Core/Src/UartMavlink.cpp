#include "UartMavlink.h"

/********************************************************************************************
 * Transmit
 */
static volatile uint8_t mavlink_tx_busy = 0;

void mavlink_log(const std::string& log_msg, MAV_SEVERITY severity) {
	// Check if busy
	if (mavlink_tx_busy) return;

	// Check if string is too long
	if (log_msg.size() > 50) {
		mavlink_log("[WARN]: MAVLink log message length is too long", MAV_SEVERITY_WARNING);
		return;
	}

	// Make message
	mavlink_message_t mavlink_log_msg;
	static uint8_t mavlink_log_buffer[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_statustext_pack(0, 0, &mavlink_log_msg, severity, log_msg.c_str(), 0, 0);
	uint16_t len = mavlink_msg_to_send_buffer(mavlink_log_buffer, &mavlink_log_msg);

	// Send message
	mavlink_tx_busy = 1;
	HAL_StatusTypeDef result = HAL_UART_Transmit_IT(&huart1, mavlink_log_buffer, len);

	if (result == HAL_TIMEOUT) {
		char local_log_buf[100];
		int len = snprintf(local_log_buf, sizeof(local_log_buf), "[WARN]: MAVLink Log Timeout");
		HAL_UART_Transmit(&huart2, (uint8_t*)local_log_buf, len, 100);
	} else if (result != HAL_OK) {
		char local_log_buf[100];
		int len = snprintf(local_log_buf, sizeof(local_log_buf), "[ERROR]: MAVLink Log Failed");
		HAL_UART_Transmit(&huart2, (uint8_t*)local_log_buf, len, 100);
	} else {
		mavlink_tx_busy = 0;
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        mavlink_tx_busy = 0;
    }
}


/********************************************************************************************
 * Receive
 */


