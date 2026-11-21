#ifndef INC_REAL_MAIN_H_
#define INC_REAL_MAIN_H_


#ifdef __cplusplus
#include "throttleController.h"
extern "C"
{
	#include "dshot.h"
#endif


int real_main(float* motor_values);

#ifdef __cplusplus
}
#endif


#endif /* INC_REAL_MAIN_H_ */
