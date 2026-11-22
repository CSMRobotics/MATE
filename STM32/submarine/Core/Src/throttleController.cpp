#include "throttleController.h"

bool ThrottleController::transmit(float* normals) {
	uint16_t my_motor_value[8];

	for (size_t i = 0; i < 8; ++i) {
		my_motor_value[i] = motorValueFromFloat(normals[i]);
	}

	dshot_write(my_motor_value);
	return true;
}

