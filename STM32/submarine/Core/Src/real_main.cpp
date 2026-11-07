#include "real_main.h"
#include "throttleController.h"

extern "C" {
	#include "dshot.h"
}

int setup()
{

}

int real_main()
{
	float test_vals[8] = {0.99, 0.5, 0.7, 0.8, -1.0, -0.8, 0.2, -0.3};
	ThrottleController tc();

	tc.transmit(test_vals);
}
