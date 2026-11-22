#ifndef THROTTLE_CONTROLLER_H
#define THROTTLE_CONTROLLER_H

/**
 * @brief Just a mock class that prints thruster values for now
 */
class ThrottleController
{
public:
	ThrottleController() = default;
	bool transmit(float* actual);

private:
};
#endif
