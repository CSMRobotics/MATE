#include <rclcpp/rclcpp.hpp>
#include "pca9685/PCA9685_Node.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCA9685_Node>());
    rclcpp::shutdown();
    return 0;
}