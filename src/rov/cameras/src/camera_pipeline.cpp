#include "cameras/camera_pipeline.hpp"

#include "gst/gst.h"

#include <cstdio>
#include <unordered_map>

#define FRONT_CAMERA_ARGUS_SENSOR_ID 0
#define TRANSECT_CAMERA_ARGUS_SENSOR_ID 1

#define PIPELINE_TEST "\"videotestsrc is-live=true ! x264enc speed-preset=ultrafast tune=zerolatency ! rtph264pay name=pay0 pt=96\""
#define PIPELINE_FRONT "\"nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96\""
#define PIPELINE_TRANSECT "\"nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96\""

namespace {
    std::unordered_map<Camera, std::string> uris = {
        {Camera::MIPI_0, "rtsp://localhost:8554/rov_front"},
        {Camera::MIPI_1, "rtsp://localhost:8554/rov_transect"},
        {Camera::USB_0, "rtsp://localhost:8554/usb_0"},
        {Camera::USB_1, "rtsp://localhost:8554/usb_1"},
    };

    std::unordered_map<Camera, std::string> mount_strs = {
        {Camera::MIPI_0, "/rov_front"},
        {Camera::MIPI_1, "/rov_transect"},
        {Camera::USB_0, "/usb_0"},
        {Camera::USB_1, "/usb_1"}
    };
}

GstElement* CameraStreamFactory::create_element([[maybe_unused]] GstRTSPMediaFactory *factory, [[maybe_unused]] const GstRTSPUrl *url) {
    switch(camera) {
        case Camera::MIPI_0:
            return gst_parse_launch(
                PIPELINE_FRONT, nullptr
            );
            break;
        case Camera::MIPI_1:
            return gst_parse_launch(
                PIPELINE_TRANSECT, nullptr
            );
            break;
        case Camera::USB_0:
            // TODO: implement
            return gst_parse_launch(
                PIPELINE_TEST, nullptr
            );
            break;
        case Camera::USB_1:
            // TODO: implement
            return gst_parse_launch(
                PIPELINE_TEST, nullptr
            );
            break;
        default:
            // if no specified camera, return default test stream source
            return gst_parse_launch(
                PIPELINE_TEST, nullptr);
            break;
    }
}

CameraStream::CameraStream(Camera camera, int argc, char** argv) {
    gst_thread = std::thread(std::bind(&CameraStream::run_dummy_stream, this, camera, argc, argv));
    gst_thread.detach();
}

void media_configure([[maybe_unused]] GstRTSPMediaFactory *factory, GstRTSPMedia *media, [[maybe_unused]] gpointer user_data) {
    GstElement *pipeline = gst_rtsp_media_get_element(media);
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_object_set_data(G_OBJECT(media), "pipeline", pipeline);
}

void handle_client_command([[maybe_unused]] GstRTSPClient *client, GstRTSPContext *ctx, [[maybe_unused]] gpointer user_data) {
    GstRTSPMedia *media = ctx->media;
    GstElement *pipeline = static_cast<GstElement*>(g_object_get_data(G_OBJECT(media), "pipeline"));

    if (ctx->request) {
        GstRTSPMethod method = ctx->method;
        if (method == GST_RTSP_PAUSE) {
            gst_element_set_state(pipeline, GST_STATE_PAUSED);
        } else if (method == GST_RTSP_PLAY) {
            gst_element_set_state(pipeline, GST_STATE_PLAYING);
        }
    }
}

void CameraStream::run_stream(Camera camera, int argc, char** argv) {
    gst_init(&argc, &argv);

    GstRTSPServer* server = gst_rtsp_server_new();
    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);

    CameraStreamFactory* factory = new CameraStreamFactory(camera);
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure), nullptr);

    gst_rtsp_mount_points_add_factory(mounts, mount_strs[camera].c_str(), factory);

    GstRTSPClient* client = gst_rtsp_client_new();
    g_signal_connect(client, "client-control-request", G_CALLBACK(handle_client_command), nullptr);
    gst_rtsp_server_attach(server, nullptr);

    g_main_loop_run(g_main_loop_new(nullptr, false));
}

void CameraStream::run_dummy_stream([[maybe_unused]] Camera camera, int argc, char** argv) {
    rclcpp::init(argc, argv);
    gst_init(&argc, &argv);

    GstElement* pipeline = gst_parse_launch(PIPELINE_TEST, nullptr);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    GstBus* bus = gst_element_get_bus(pipeline);
    GstMessage* msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

    if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
        // RCLCPP_ERROR(rclcpp::get_logger(), "Error occured during the stream");
    }

    gst_message_unref(msg);
    gst_object_unref(bus);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
    rclcpp::shutdown();
}

CameraManager::CameraManager() : Node("camera_manager") {
    _camera_stream_requests = this->create_service<rov_interfaces::srv::CameraStreamCommand>("rov_camera_commands", 
        std::bind(&CameraManager::handleRequest, this, std::placeholders::_1, std::placeholders::_2));
}

CameraManager::~CameraManager() {
    for (auto& camera : cameras) {
        sendRTSPCommand(camera.first, Command::STOP);
    }
}

int CameraManager::argc = 0;
char** CameraManager::argv = nullptr;
CameraManager* CameraManager::getInstance(int _argc, char** _argv) {
    static CameraManager instance = CameraManager();
    argc = _argc;
    argv = _argv;
    return &instance;
}

bool CameraManager::playStream(Camera camera) {
    if (cameras.count(camera)) {
        // return sendRTSPCommand(camera, Command::PLAY);
    }
    return false;
}

bool CameraManager::pauseStream(Camera camera) {
    if (cameras.count(camera)) {
        // return sendRTSPCommand(camera, Command::PAUSE);
    }
    return false;
}

bool CameraManager::stopStream(Camera camera) {
    if (cameras.count(camera)) {
        // return sendRTSPCommand(camera, Command::STOP);
    }
    return false;
}

bool CameraManager::startStream(Camera camera) {
    if (cameras.count(camera)) {
        return false;
    }
    
    cameras[camera] = new CameraStream(camera, argc, argv);
    return true;
}

bool CameraManager::sendRTSPCommand(Camera camera, Command command) {
    GstRTSPUrl* url;
    GstRTSPMessage* message;
    GstRTSPConnection* connection;
    GstRTSPResult res;

    // create the rtsp connection
    gst_rtsp_url_parse(uris[camera].c_str(), &url);
    res = gst_rtsp_connection_create(url, &connection);
    if (res != GST_RTSP_OK || !connection) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to RTSP server");
        return false;
    }

    // connect to the rtsp server
    res = gst_rtsp_connection_connect(connection, nullptr);
    if (res != GST_RTSP_OK) {
        RCLCPP_ERROR(this->get_logger(), "Failed to establish RTSP connection");
        gst_rtsp_connection_close(connection);
        return false;
    }

    // create RTSP request
    GstRTSPMethod method;
    switch (command) {
        case Command::START:
        case Command::PLAY:
            method = GST_RTSP_PLAY;
            break;
        case Command::PAUSE:
            method = GST_RTSP_PAUSE;
            break;
        case Command::STOP:
            method = GST_RTSP_TEARDOWN;
            break;
    }
    gst_rtsp_message_new_request(&message, method, uris[camera].c_str());

    // send the RTSP command
    res = gst_rtsp_connection_send(connection, message, nullptr);
    if (! res == GST_RTSP_OK) {
        std::string msg = "Failed to send " + std::string(command == Command::PLAY ? "play" : "pause") + "  command";
        RCLCPP_ERROR(this->get_logger(), msg.c_str());
        return false;
    }

    // cleanup
    gst_rtsp_message_unset(message);
    gst_rtsp_connection_close(connection);

    return true;
}

void CameraManager::handleRequest(const rov_interfaces::srv::CameraStreamCommand_Request::SharedPtr request, rov_interfaces::srv::CameraStreamCommand_Response::SharedPtr response) {
    Camera camera;
    switch (request->camera_id) {
        case static_cast<uint8_t>(Camera::MIPI_0):
            camera = Camera::MIPI_0;
            break;
        case static_cast<uint8_t>(Camera::MIPI_1):
            camera = Camera::MIPI_1;
            break;
        case static_cast<uint8_t>(Camera::USB_0):
            camera = Camera::USB_0;
            break;
            case static_cast<uint8_t>(Camera::USB_1):
            camera = Camera::USB_1;
            break;
    }

    switch (request->command) {
        case static_cast<uint8_t>(Command::PLAY):
            RCLCPP_INFO(this->get_logger(), "PLAYING STREAM");
            // response->result = playStream(camera);
            break;
        case static_cast<uint8_t>(Command::PAUSE):
            RCLCPP_INFO(this->get_logger(), "PAUSING STREAM");
            // response->result = pauseStream(camera);
            break;
        case static_cast<uint8_t>(Command::STOP):
            RCLCPP_INFO(this->get_logger(), "STOPPING STREAM");
            // response->result = stopStream(camera);
            break;
        case static_cast<uint8_t>(Command::START):
            RCLCPP_INFO(this->get_logger(), "STARTING STREAM");
            response->result = startStream(camera);
            break;
    }
}