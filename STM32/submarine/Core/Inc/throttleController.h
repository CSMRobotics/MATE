#ifndef THROTTLE_CONTROLLER_H
#define THROTTLE_CONTROLLER_H

#include "dshot.h"

class ThrottleController
{
public:
	ThrottleController();
	bool transmit(float* actual);

private:
};
#endif
