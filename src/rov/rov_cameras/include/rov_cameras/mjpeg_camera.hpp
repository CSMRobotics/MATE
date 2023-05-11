#ifndef MJPEG_CAMERA_INCLUDED_HPP
#define MJPEG_CAMERA_INCLUDED_HPP

#include <rclcpp/node.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/videoio.hpp>
#include <opencv2/core/types.hpp>

#define PIPELINE_F "v4l2src device=%s io-mode=2 ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegparse ! jpegdec ! nvvidconv ! video/x-raw,format=I420 ! appsink max-buffers=1 drop=true"

class MJPEG_Camera : public rclcpp::Node {
public:
    MJPEG_Camera(std::string device, uint8_t pub_id);
private:
    void poll();

    cv::Mat img_i420 = cv::Mat(240*3/2, 320, CV_8UC1);
    cv::Mat img_bgr = cv::Mat(240,320,CV_8UC3);
    cv::VideoCapture cap;
    rclcpp::TimerBase::SharedPtr poll_func;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub;
};

#endif
