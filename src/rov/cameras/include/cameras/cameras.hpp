#ifndef CAMERAS_HPP
#define CAMERAS_HPP

#include "gst/gst.h"

#include <thread>
#include <unordered_map>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

enum class CameraSensor : uint8_t {
    MIPI_0 = 0,
    MIPI_1 = 1,
    TEST = 255
};

class Camera {
public:
    Camera() = default;
    Camera(int argc, char** argv, CameraSensor sensorID);
private:
    void run_rtsp_stream(int argc, char** argv, CameraSensor sensorID);

    std::thread gst_rtsp_server_thread;
};

class CameraManager {
public:
    static CameraManager* getInstance(int argc, char** argv);

    bool addCamera(CameraSensor sensorID);
private:
    CameraManager(int argc, char** argv);

    int argc;
    char** argv;

    std::unordered_map<CameraSensor, Camera> cameras;
};

class CameraNode : public rclcpp::Node {
public:
    CameraNode(int argc, char** argv);
private:
    CameraManager* manager;
};


#endif