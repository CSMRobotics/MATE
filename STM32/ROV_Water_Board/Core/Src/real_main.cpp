#include "real_main.h"
#include "main.h"
#include "mavlink/rov_interfaces/mavlink.h"
#include "UartMavlink.h"
#include "throttleController.h"
#include "FlightController.h"
#include "SubControls.h"

#include <string>
#include <Eigen/Geometry>

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

ThrottleController throttle_controller;

Eigen::Quaternionf q_identity = Eigen::Quaternionf::Identity();
FlightController flight_controller(
    100,
    0,
    q_identity,
    1, 1, 1,
    1, 1, 1,
    3,
    0.2,
    &throttle_controller
);

bool safe_mode = true;

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

	// Start in safe mode
	safe_mode = true;
	mavlink_log("Flight Controller start in safe mode", MAV_SEVERITY_INFO);
}

int real_main() {
	// Build sub controls from joystick commands
	SubControls sub_controls;
	sub_controls.velocityX = latest_sub_controls.velocity_x;
	sub_controls.velocityY = latest_sub_controls.velocity_y;
	sub_controls.velocityZ = latest_sub_controls.velocity_z;

	sub_controls.omegaX = latest_sub_controls.omega_x;
	sub_controls.omegaY = latest_sub_controls.omega_y;
	sub_controls.omegaZ = latest_sub_controls.omega_z;

	sub_controls.mode_switch = latest_sub_controls.mode_switch;
	sub_controls.estop = latest_sub_controls.estop;

	// Watchdog the heartbeat
	if (HAL_GetTick() - last_mavlink_heartbeat_time > 1000 && !safe_mode) {
		sub_controls.estop = true;
		safe_mode = true;
		mavlink_log("ESTOP: Heartbeat disconnection", MAV_SEVERITY_ERROR);
	}

	// Test Flight Controller loop
	if (!safe_mode) {
		flight_controller.update_orient_state(Eigen::Quaternionf::Identity());
		flight_controller.update_depth_state(0);
		flight_controller.update_sub_controls(sub_controls);
		flight_controller.update_simple();
	}


	// Estop
	if (sub_controls.estop) {
		safe_mode = true;
		flight_controller.activate_safe_mode();
		mavlink_log("ESTOP: Activated safe mode", MAV_SEVERITY_WARNING);
	}

	// Switch off safe mode
	if (sub_controls.mode_switch) {
		safe_mode = false;
		flight_controller.disable_safe_mode();
		mavlink_log("Mode switch: Deactivate safe mode", MAV_SEVERITY_INFO);
	}

	HAL_Delay(100);
    return 0;
}
