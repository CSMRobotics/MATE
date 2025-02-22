#include <rclcpp/rclcpp.hpp>
#include <flight_controller/controller.hpp>

#include <csignal>

void signal_handler(int signal) {
    try {
        if (signal == SIGINT) {
            rclcpp::shutdown();
        }
    } catch (const rclcpp::exceptions::RCLError& err) {
        rcutils_reset_error();
    }
}

int main(int argc, char ** argv) {
    std::signal(SIGINT, signal_handler);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<FlightController>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    
    return 0;
}