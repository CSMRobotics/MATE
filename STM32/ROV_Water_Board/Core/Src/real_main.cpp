#include "cmsis_os.h"
#include "real_main.h"
#include "main.h"
#include "mavlink/common/mavlink.h"

extern UART_HandleTypeDef huart1; // defined in main.c so i think you have to do extern

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

int real_main() {
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // turn led on before transmitting
	mavlink_tx_heartbeat();
	osDelay(50); // give it a lil bit
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // turn led back off (after transmitting)
	osDelay(950); // wait some more

	return 0;
}
