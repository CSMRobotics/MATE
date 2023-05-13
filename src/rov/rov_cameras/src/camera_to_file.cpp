#include "camera_to_file.hpp"

File_Camera::File_Camera(const std::string& device, uint8_t id, const std::string& file_name, bool mjpeg) : rclcpp::Node(std::string("file_camera") + std::to_string(id)) {
    char command[1025] = {0};

    if(mjpeg)
        snprintf(command, sizeof(command)-1, PIPELINE_MJPEG_F, device.c_str(), file_name.c_str());
    else
        snprintf(command, sizeof(command)-1, PIPELINE_H264_F, device.c_str(), file_name.c_str());


    auto pipeline = [&](){
        system(command);
    };

    thread = std::thread(pipeline);
}

File_Camera::~File_Camera() {
    thread.~thread();
}