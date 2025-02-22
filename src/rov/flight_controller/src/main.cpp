#include <rclcpp/rclcpp.hpp>
#include <flight_controller/40.hpp>

#include <csignal>

void signal_handler(int signal) {
    if (signal == SIGINT) {
        rclcpp::shutdown();
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