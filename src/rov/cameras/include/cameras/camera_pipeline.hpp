#ifndef CAMERA_PIPELINE_HPP
#define CAMERA_PIPELINE_HPP

#include <thread>
#include <mutex>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

#include <gst/rtsp-server/rtsp-server.h>

#include "rov_interfaces/srv/camera_stream_command.hpp"

class CameraManager;

enum class Camera : uint8_t {
    MIPI_0 = 0,
    MIPI_1 = 1,
    USB_0 = 2,
    USB_1 = 3
};

class CameraStreamFactory : public GstRTSPMediaFactory {
public:
    CameraStreamFactory(Camera camera) : camera(camera) {};
protected:
    GstElement* create_element(GstRTSPMediaFactory *factory, const GstRTSPUrl *url);
private:
    Camera camera;
};

class CameraStream {
public:
    CameraStream(Camera camera, int argc, char** argv);
private:
    void run_stream(Camera camera, int argc, char** argv);
    void run_dummy_stream(Camera camera, int argc, char** argv);

    std::thread gst_thread;
friend CameraManager;
};

class CameraManager : public rclcpp::Node {
public:
    ~CameraManager() noexcept;

    // delete copy constructor and copy assignment operator
    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    static CameraManager* getInstance(int argc, char** argv);

    bool playStream(Camera camera);
    bool pauseStream(Camera camera);
    bool stopStream(Camera camera);
    bool startStream(Camera camera);
private:
    CameraManager();

    enum class Command : uint8_t {
        PLAY = 0,
        PAUSE = 1,
        STOP = 2,
        START = 3
    };
    bool sendRTSPCommand(Camera camera, Command command);

    void handleRequest(const rov_interfaces::srv::CameraStreamCommand_Request::SharedPtr request, rov_interfaces::srv::CameraStreamCommand_Response::SharedPtr response);

    rclcpp::Service<rov_interfaces::srv::CameraStreamCommand>::SharedPtr _camera_stream_requests;

    std::unordered_map<Camera, CameraStream*> cameras;

    static int argc;
    static char** argv;
};

#endif
