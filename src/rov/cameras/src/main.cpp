#include "cameras/cameras.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<CameraNode>(argc, argv));

    rclcpp::shutdown();
}