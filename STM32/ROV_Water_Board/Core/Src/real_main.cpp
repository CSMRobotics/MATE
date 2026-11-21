#include "cmsis_os.h"
#include "real_main.h"
#include "main.h"
#include "mavlink/rov_interfaces/mavlink.h"
#include "UartMavlink.h"

#include <string>

extern UART_HandleTypeDef huart1; // defined in main.c so i think you have to do extern
extern UART_HandleTypeDef huart2;

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




static mavlink_message_t mavlink_msg;
static mavlink_status_t mavlink_status;
volatile uint8_t sub_controls_received = 0;
mavlink_sub_controls_t latest_sub_controls;
uint8_t rx_byte;

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

void setup() {
	HAL_UART_Transmit(
		&huart2,
		(uint8_t*)"Interrupt RX Started\r\n",
		23,
		100
	);
	mavlink_log("Interrupt RX Started", MAV_SEVERITY_INFO);

	// Start receiving one byte using interrupts
	HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

}

int real_main() {
	// Log the velocity joystick command
	static uint32_t last_log_ms = 0;

	if (sub_controls_received) {
		sub_controls_received = 0;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		uint32_t now = HAL_GetTick();
		if (now - last_log_ms >= 100) {   // 4 Hz
			last_log_ms = now;
			char log_buf[50];
			std::snprintf(log_buf, sizeof(log_buf), "Vel: (%.2f, %.2f, %.2f)",
				latest_sub_controls.velocity_x,
				latest_sub_controls.velocity_y,
				latest_sub_controls.velocity_z);
			mavlink_log(log_buf, MAV_SEVERITY_INFO);
		}
	}

    return 0;
}
