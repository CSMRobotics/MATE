#include "flight_controller/40.hpp"

FlightController::FlightController() : Node("flight_controller") {
    this->create_wall_timer(std::chrono::milliseconds(16), std::bind(&FlightController::updatePID, this));
}

void FlightController::updatePID() {
    const float dt = 0.016; // 16 milliseconds per update, TODO: replace with dynamic measurement?
    RCLCPP_INFO(this->get_logger(), "update");
}

void FlightController::updateJoy() {
    
}