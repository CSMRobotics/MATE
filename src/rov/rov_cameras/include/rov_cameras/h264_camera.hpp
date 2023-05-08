#ifndef H264_CAMERA_INCLUDED_HPP
#define H264_CAMERA_INCLUDED_HPP

#include <rclcpp/rclcpp.hpp>

class H264_Camera : public rclcpp::Node {
public:
    H264_Camera(std::string device, uint8_t pub_id);
private:
};

#endif