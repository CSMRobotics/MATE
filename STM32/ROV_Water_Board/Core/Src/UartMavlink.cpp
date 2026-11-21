#include "UartMavlink.h"

//********************************************************************************************
// Transmit
static volatile uint8_t mavlink_tx_busy = 0;

void mavlink_log(const std::string& log_msg, MAV_SEVERITY severity) {
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
	HAL_StatusTypeDef result = HAL_UART_Transmit(&huart1, mavlink_log_buffer, len, 10);

	if (result == HAL_TIMEOUT) {
		char local_log_buf[100];
		int len = snprintf(local_log_buf, sizeof(local_log_buf), "[WARN]: MAVLink Log Timeout");
		HAL_UART_Transmit(&huart2, (uint8_t*)local_log_buf, len, 100);
	} else if (result != HAL_OK) {
		char local_log_buf[100];
		int len = snprintf(local_log_buf, sizeof(local_log_buf), "[ERROR]: MAVLink Log Failed");
		HAL_UART_Transmit(&huart2, (uint8_t*)local_log_buf, len, 100);
	}
}


//********************************************************************************************
// Receive

static mavlink_message_t mavlink_msg;
static mavlink_status_t mavlink_status;
volatile uint8_t sub_controls_received = 0;
mavlink_sub_controls_t latest_sub_controls;

uint8_t rx_byte;

void init_uart_mavlink_rx() {
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

// UART Interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        // Feed received byte into MAVLink parser
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &mavlink_msg, &mavlink_status)) {
            if (mavlink_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                // Heartbeat received
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            }

            else if (mavlink_msg.msgid == MAVLINK_MSG_ID_SUB_CONTROLS) {
                mavlink_msg_sub_controls_decode(
                    &mavlink_msg,
                    &latest_sub_controls
                );

                sub_controls_received = 1;
            }
        }

        // Restart reception for the next byte
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_AbortReceive(huart);          // forces gState/RxState back to READY
        HAL_UART_Receive_IT(huart, &rx_byte, 1);
    }
}
