#include "rov_cameras/h264_camera.hpp"

H264_Camera::H264_Camera([[maybe_unused]] std::string device, [[maybe_unused]] uint8_t pub_id) : rclcpp::Node("camera_mjpeg" + static_cast<char>(pub_id+48)) {

}