#include "manipulator_controller/manipulator_controller.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManipulatorController>());
    rclcpp::shutdown();
    return 0;
}