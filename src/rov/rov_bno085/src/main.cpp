#include "rov_bno085/bno08x_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BNO08X_Node>());
    rclcpp::shutdown();
    return 0;
}