#include "real_main.h"
#include "dshot.h"

int real_main(float* motor_values)
{
	ThrottleController tc;

//	float motorValueFloat = 0.0f;
//	uint16_t motorValue = motorValueFromFloat(motorValueFloat);
//	uint16_t my_motor_value[8];
//	my_motor_value[0] = motorValue;
//	my_motor_value[1] = motorValue;
//	my_motor_value[2] = motorValue;
//	my_motor_value[3] = motorValue;
//
//	my_motor_value[4] = motorValue;
//	my_motor_value[5] = motorValue;
//	my_motor_value[6] = motorValue;
//	my_motor_value[7] = motorValue;
//
//	dshot_write(my_motor_value);
	tc.transmit(motor_values);
	return 1;
}
