#include "fast_sensor_fusion/SensorFusion.h"

#include <signal.h>

bool RUNNING = true;

/**
 * @brief Handles SIGINT for clean shutdown
 */
void signal_handler(int signum) {
    if (signum == SIGINT || signum == SIGTERM) {
        std::cout << "Received SIGINT, shutting down...\n";
        RUNNING = false;
    }   
}

int main(int argc, char * argv[]) {
    signal(SIGINT, signal_handler);
    rclcpp::init(argc, argv);
    SensorFusion sensor_fusion = SensorFusion();

    while (RUNNING) {
        sensor_fusion.update();
    }
    sensor_fusion.shutdown();
    
    rclcpp::shutdown();
    return 0;
}
