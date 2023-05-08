#ifndef H264_CAMERA_INCLUDED_HPP
#define H264_CAMERA_INCLUDED_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <h264_msgs/msg/packet.hpp>

#include <thread>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavdevice/avdevice.h>
}

class H264_Camera : public rclcpp::Node {
public:
    H264_Camera(std::string device, uint8_t pub_id);
    ~H264_Camera() override;
private:
    void restart();

    int seq{};
    AVInputFormat* format = nullptr;
    AVFormatContext* context = nullptr;
    std::thread cam_thread;
    std::atomic<bool> stop = false;

    std::string device;

    sensor_msgs::msg::CameraInfo camera_info_msg;
    rclcpp::Publisher<h264_msgs::msg::Packet>::SharedPtr h264_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

};

#endif