#include "rov_cameras/mjpeg_camera.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

MJPEG_Camera::MJPEG_Camera(std::string device, uint8_t pub_id) : rclcpp::Node(std::string("camera_mjpeg") + std::to_string(static_cast<int>(pub_id))) {
    image_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>(std::string("camera_mjpeg") + std::to_string(static_cast<int>(pub_id)) + std::string("/image_raw/compressed"), 10);
    poll_func = this->create_wall_timer(std::chrono::milliseconds(1000/30), std::bind(&MJPEG_Camera::poll, this));

    cv::Mat yComponent(240, 320, CV_8UC1);
    cv::Mat uComponent(240/2, 320/2, CV_8UC1);
    cv::Mat vComponent(240/2, 320/2, CV_8UC1);
    std::vector<cv::Mat> yuvComponents = {yComponent, uComponent, vComponent};

    cv::merge(yuvComponents, img_i420);

    char pipeline[1025] = {0};
    snprintf(pipeline, sizeof(pipeline)-1, PIPELINE_F, device.c_str());
    cap = cv::VideoCapture(std::string(pipeline), cv::CAP_GSTREAMER);
}

void MJPEG_Camera::poll() {
    cap.grab();
    cap.retrieve(this->img_i420);
    cv::cvtColor(this->img_i420, this->img_bgr, cv::COLOR_YUV2BGR_NV12);
    sensor_msgs::msg::CompressedImage msg;
    std_msgs::msg::Header header;
    header.set__stamp(this->now());
    cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, this->img_bgr);
    img_bridge.toCompressedImageMsg(msg);
    image_pub->publish(msg);
}