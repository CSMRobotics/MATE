#ifndef CAMERA_TO_FILE_INCLUDED_HPP
#define CAMERA_TO_FILE_INCLUDED_HPP

#include <rclcpp/node.hpp>
#include <stdio.h>
#include <thread>

#define PIPELINE_H264_F "gst-launch-1.0 v4l2src device=%s ! nvv4l2decoder ! nvvidconv ! mp4mux ! filesink location=%s"
#define PIPELINE_MJPEG_F "gst-launch-1.0 v4l2src device=%s io-mode=2 ! image/jpeg,width=320,height=240,framerate=30/1 ! jpegparse ! jpegdec ! jpegenc ! avimux ! filesink location=%s"

class File_Camera : public rclcpp::Node {
public:
    File_Camera(const std::string& device, uint8_t id, const std::string& file_name, bool mjpeg = false);
    ~File_Camera() override;
private:
    std::thread thread;
};

#endif