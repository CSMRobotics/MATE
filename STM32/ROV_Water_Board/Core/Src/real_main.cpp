#include "real_main.h"
#include "main.h"
#include "mavlink/rov_interfaces/mavlink.h"
#include "UartMavlink.h"

#include <string>

extern UART_HandleTypeDef huart1; // defined in main.c so i think you have to do extern
extern UART_HandleTypeDef huart2;

void log_joy_command() {
	// Log the joystick command
	static uint32_t last_log_ms = 0;

	if (sub_controls_received) {
		sub_controls_received = 0;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		uint32_t now = HAL_GetTick();
		if (now - last_log_ms >= 100) {   // 10 Hz
			last_log_ms = now;
			char log_buf[50];
			std::snprintf(log_buf, sizeof(log_buf), "Vel: (%.2f, %.2f, %.2f)",
				latest_sub_controls.velocity_x,
				latest_sub_controls.velocity_y,
				latest_sub_controls.velocity_z);
			mavlink_log(log_buf, MAV_SEVERITY_INFO);

			std::snprintf(log_buf, sizeof(log_buf), "Rot: (%.2f, %.2f, %.2f)",
				latest_sub_controls.omega_x,
				latest_sub_controls.omega_y,
				latest_sub_controls.omega_z);
			mavlink_log(log_buf, MAV_SEVERITY_INFO);

			std::snprintf(log_buf, sizeof(log_buf), "Mode Switch: %u, Estop: %u",
				latest_sub_controls.mode_switch,
				latest_sub_controls.estop);
			mavlink_log(log_buf, MAV_SEVERITY_INFO);
		}
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
	init_uart_mavlink_rx();

}

int real_main() {

	log_joy_command();
    return 0;
}
