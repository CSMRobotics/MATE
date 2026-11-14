#include "cmsis_os.h"
#include "real_main.h"
#include "main.h"
#include "mavlink/common/mavlink.h"

extern UART_HandleTypeDef huart1; // defined in main.c so i think you have to do extern
extern UART_HandleTypeDef huart2;

void setup() {
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

void mavlink_tx_heartbeat(void) {
	static mavlink_message_t msg;
	static uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

	mavlink_msg_heartbeat_pack(
			1, // system id
			1, // component id
			&msg, // message
			MAV_TYPE_SUBMARINE, // its a sub
			MAV_AUTOPILOT_GENERIC,   // Autopilot type
			MAV_MODE_PREFLIGHT,      // System mode
			0,                       // Custom mode
			MAV_STATE_STANDBY        // System status
	);
	uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

	HAL_UART_Transmit(&huart1, buffer, len, 1000);
}

//void mavlink_rx_poll(void) {
//	uint8_t rx_byte;
//	static mavlink_message_t msg;
//	static mavlink_status_t status;
//
//	// Drain all available bytes in huart1 (timeout = 0 for non-blocking)
//	while (HAL_UART_Receive(&huart1, &rx_byte, 1, 0) == HAL_OK) {
//		if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &msg, &status)) {
//			if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
//				char log_buf[80];
//				int log_len = snprintf(
//					log_buf, sizeof(log_buf),
//					"[STM32 RX] Heartbeat from SysID: %d | CompID: %d\r\n",
//					msg.sysid, msg.compid
//				);
//
//				HAL_UART_Transmit(&huart2, (uint8_t*)log_buf, log_len, 100);
//			}
//		}
//	}
//}



int real_main() {
//	HAL_UART_Transmit(&huart2, (uint8_t*)"started\r\n", 9, 100); // send a message to the terminal
//
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // turn led on before transmitting
//	mavlink_tx_heartbeat();
//	osDelay(50); // give it a lil bit
//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // turn led back off (after transmitting)
//	osDelay(950); // wait some more

//	---------------------------------------------------------------------------------------
	uint8_t rx_byte;
	    static mavlink_message_t msg;
	    static mavlink_status_t status;

	    HAL_UART_Transmit(&huart2, (uint8_t*)"RX Poll Started\r\n", 17, 100);

	    while (1) {
	        // Clear Overrun Error just in case a byte arrived before the loop started
	        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
	            __HAL_UART_CLEAR_OREFLAG(&huart1);
	        }

	        // Continuously check for 1 byte (non-blocking, timeout = 0)
	        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 0) == HAL_OK) {

	            // Optional: Uncomment the line below to print a dot for EVERY raw byte received
//	             HAL_UART_Transmit(&huart2, (uint8_t*)".", 1, 10);

	            if (mavlink_parse_char(MAVLINK_COMM_0, rx_byte, &msg, &status)) {
	                if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
	                    char log_buf[80];
	                    int log_len = snprintf(
	                        log_buf, sizeof(log_buf),
	                        "[STM32 RX] Heartbeat from SysID: %d | CompID: %d\r\n",
	                        msg.sysid, msg.compid
	                    );

	                    HAL_UART_Transmit(&huart2, (uint8_t*)log_buf, log_len, 100);
	                }
	            }
	        }
	    }

	return 0;
}
