#include "cameras/cameras.hpp"

#include <functional>
#include "gst/rtsp-server/rtsp-server.h"

#define PIPELINE_TEST "videotestsrc is-live=true ! x264enc speed-preset=ultrafast tune=zerolatency ! rtph264pay name=pay0 pt=96"
#define PIPELINE_FRONT "\"nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96\""
#define PIPELINE_TRANSECT "\"nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM),width=1920,height=1080 ! nvvidconv ! nvv4l2h264enc ! h264parse ! rtph264pay name=pay0 pt=96\""

namespace {
    std::unordered_map<CameraSensor, std::string> ports = {
        {CameraSensor::MIPI_0, "8554"},
        {CameraSensor::MIPI_1, "8555"},
        {CameraSensor::TEST, "8809"}
    };
}

Camera::Camera(int argc, char** argv, CameraSensor sensorID) {
    gst_rtsp_server_thread = std::thread(std::bind(&Camera::run_rtsp_stream, this, argc, argv, sensorID));
    gst_rtsp_server_thread.detach();
}

void Camera::run_rtsp_stream(int argc, char** argv, CameraSensor sensorID) {
    GMainLoop *loop;
    GstRTSPServer *server;
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    gst_init(&argc, &argv);

    // create main loop to spin the server
    loop = g_main_loop_new(NULL, false);

    // create new server
    server = gst_rtsp_server_new();
    g_object_set(server, "service", ports[sensorID].c_str(), NULL);

    // get mount points to map uri to media factory
    mounts = gst_rtsp_server_get_mount_points(server);

    // media factory to create pipeline, enables rtcp
    factory = gst_rtsp_media_factory_new();

    // apply the pertinent pipeline
    switch (sensorID) {
        case CameraSensor::MIPI_0:
            gst_rtsp_media_factory_set_launch(factory, PIPELINE_FRONT);
            break;
        case CameraSensor::MIPI_1:
            gst_rtsp_media_factory_set_launch(factory, PIPELINE_TRANSECT);
            break;

        case CameraSensor::TEST:
        default:
            gst_rtsp_media_factory_set_launch(factory, PIPELINE_TEST);
            break;
    }

    gst_rtsp_media_factory_set_shared(factory, true);
    gst_rtsp_media_factory_set_enable_rtcp(factory, true);

    // attach this factory to the mount point
    gst_rtsp_mount_points_add_factory(mounts, "/video", factory);

    // dont need the pointer anymore
    g_object_unref(mounts);

    // attach server to default context
    gst_rtsp_server_attach(server, NULL);

    // serve that server
    std::string msg = "Now serving RTSP video stream on 127.0.0.1:" + ports[sensorID] + "/video";
    RCLCPP_INFO(rclcpp::get_logger("rtsp_streamer") , msg.c_str());
    g_main_loop_run(loop);
}

CameraManager* CameraManager::getInstance(int argc, char** argv) {
    static CameraManager instance{argc, argv};
    return &instance;
}

CameraManager::CameraManager(int argc, char** argv) : argc(argc), argv(argv) {
    
}

bool CameraManager::addCamera(CameraSensor sensorID) {
    if (cameras.count(sensorID)) {
        RCLCPP_WARN(rclcpp::get_logger("rtsp_streamer"),"Attempted to overwrite sensor, ignoring");
        return false;
    }
    
    cameras[sensorID] = Camera(argc, argv, sensorID);
    return true;
}

CameraNode::CameraNode(int argc, char** argv) : Node("rov_cameras") {
    manager = CameraManager::getInstance(argc, argv);

    manager->addCamera(CameraSensor::MIPI_0);
    manager->addCamera(CameraSensor::MIPI_1);
    manager->addCamera(CameraSensor::TEST);
}