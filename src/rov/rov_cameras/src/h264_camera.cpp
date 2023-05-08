#include "rov_cameras/h264_camera.hpp"

H264_Camera::H264_Camera([[maybe_unused]] std::string device, [[maybe_unused]] uint8_t pub_id) : rclcpp::Node(std::string("camera_h264" + std::to_string(static_cast<int>(pub_id)))) {

}