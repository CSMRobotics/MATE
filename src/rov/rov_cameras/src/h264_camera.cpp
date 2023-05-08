#include "rov_cameras/h264_camera.hpp"

#include <camera_calibration_parsers/parse.hpp>

namespace {
void declareParameters(rclcpp::Node* node) {

}
}

H264_Camera::H264_Camera(std::string device, uint8_t pub_id) : rclcpp::Node(std::string("camera_h264") + std::to_string(static_cast<int>(pub_id))) {
    avdevice_register_all();
    
    format = av_find_input_format("video4linux2");
    if(!format) {
        RCLCPP_ERROR(this->get_logger(), "Could not find v4l2 driver");
    }

    context = avformat_alloc_context();
    if(!context) {
        RCLCPP_ERROR(this->get_logger(), "Could not allocate format context");
    }

    // declare parameters
    declareParameters(this);

    // create publisher
    this->device = device;
    h264_pub = this->create_publisher<h264_msgs::msg::Packet>(std::string("camera_h264") + std::to_string(static_cast<int>(pub_id)) + std::string("/image_raw/h264"), 10);
    camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(std::string("camera_h264") + std::to_string(static_cast<int>(pub_id)) + std::string("/camera_info"), 10);

    restart();
}

void H264_Camera::restart() {
    // if thread can be joined, join it
    if(cam_thread.joinable()) {
        stop = true;
        cam_thread.join();
        stop = false;
    }

    // set format options
    AVDictionary* format_options = nullptr; // we do not own this pointer
    av_dict_set(&format_options, "input_format", "h264", 0);
    av_dict_set(&format_options, "framerate", "30", 0);
    av_dict_set(&format_options, "video_size", "320x240", 0);

    // open h264 v4l device
    if(avformat_open_input(&context, device.c_str(), format, &format_options) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Could not open h264 device %s", device.c_str());
    }


    auto cam_callback = [this]() {
        while(!stop && rclcpp::ok()) {
            h264_msgs::msg::Packet h264_msg;
            h264_msg.seq = this->seq++;

            // block this thread until a frame is ready
            AVPacket packet;
            if(av_read_frame(this->context, &packet) < 0) {
                // EOS
                break;
            }

            rclcpp::Time stamp = this->now();

            // save bandwidth
            if(this->h264_pub->get_subscription_count() > 0) {
                // insert h264 data into end of h264 message
                h264_msg.data.insert(h264_msg.data.end(), &packet.data[0], &packet.data[packet.size]);
                h264_msg.header.stamp = stamp;
                this->h264_pub->publish(h264_msg);
            }

            av_packet_unref(&packet);
        }

        // close v4l device
        avformat_close_input(&context);
    };

    cam_thread = std::thread(cam_callback);
}

H264_Camera::~H264_Camera() {
    if(cam_thread.joinable()) {
        stop = true;
        cam_thread.join();
    }
    avformat_free_context(context);
}