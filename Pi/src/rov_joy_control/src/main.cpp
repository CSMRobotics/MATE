#include "rov_joy_control/ROVJoyControl.h"

#include <cstdio>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<ROVJoyControl> node = std::make_shared<ROVJoyControl>();

    // Spins until death
    rclcpp::spin(node);

    node.reset();

    rclcpp::shutdown();
    return 0;
}
