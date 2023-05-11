#ifndef H264_CAMERA_INCLUDED_HPP
#define H264_CAMERA_INCLUDED_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/videoio.hpp>
#include <opencv2/core/types.hpp>

#define PIPELINE_F "v4l2src device=%s ! nvv4l2decoder ! nvvidconv ! video/x-raw,width=320,height=240,format=I420 ! appsink"

class H264_Camera : public rclcpp::Node {
public:
    H264_Camera(std::string device, uint8_t pub_id);
private:
    void poll();

    cv::Mat img_i420 = cv::Mat(240*3/2, 320, CV_8UC1);
    cv::Mat img_bgr = cv::Mat(240,320,CV_8UC3);
    cv::VideoCapture cap;

    sensor_msgs::msg::CameraInfo camera_info_msg;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr camera_pub;
    rclcpp::TimerBase::SharedPtr poll_func;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;

};

#endif