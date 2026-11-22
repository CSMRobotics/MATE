#include "throttleController.h"
#include "UartMavlink.h"

bool ThrottleController::transmit(float* normals) {
	char log_buf[50];

	std::snprintf(log_buf, sizeof(log_buf), "Front Left:      %.3f", (double)normals[0]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	std::snprintf(log_buf, sizeof(log_buf), "Front Right:     %.3f", (double)normals[1]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	std::snprintf(log_buf, sizeof(log_buf), "Back Left:       %.3f", (double)normals[2]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	std::snprintf(log_buf, sizeof(log_buf), "Back Right:      %.3f", (double)normals[3]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	std::snprintf(log_buf, sizeof(log_buf), "Mid Front Left:  %.3f", (double)normals[4]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	std::snprintf(log_buf, sizeof(log_buf), "Mid Front Right: %.3f", (double)normals[5]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	std::snprintf(log_buf, sizeof(log_buf), "Mid Back Left:   %.3f", (double)normals[6]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	std::snprintf(log_buf, sizeof(log_buf), "Mid Back Right:  %.3f", (double)normals[7]);
	mavlink_log(log_buf, MAV_SEVERITY_INFO);

	mavlink_log("", MAV_SEVERITY_INFO);

	return true;
}

