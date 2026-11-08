#include "real_main.h"

#include "FlightController.h"
#include "Eigen/Geometry"

int real_main()
{
	float test_vals[8] = {0.99, 0.5, 0.7, 0.8, -1.0, -0.8, 0.2, -0.3};
	ThrottleController tc;

	tc.transmit(test_vals);

	// Flight control loop
	SubControls sub_controls = {0, 0, 0, 0, 0, 0};
	FlightController flight_controller(0.01, -4, 2, 0.5, 0.25);
	flight_controller.update_orient_state(Eigen::Quaternionf::Identity());
	flight_controller.update_altitude_state(-4);
	flight_controller.update_sub_controls(sub_controls);
	
	return 1;
}
