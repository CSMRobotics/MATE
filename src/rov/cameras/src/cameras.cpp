#include "cameras/camera_pipeline.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<CameraManager> shared_manager(CameraManager::getInstance(argc, argv));
    rclcpp::spin(shared_manager);

    rclcpp::shutdown();
    return 0;
}