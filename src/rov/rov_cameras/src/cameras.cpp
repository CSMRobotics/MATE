#include "rov_cameras/cameras.hpp"

#include "gst/rtsp-server/rtsp-server.h"
#include <functional>
#include <rcl_interfaces/msg/parameter_descriptor.h>

Camera::Camera(int argc, char **argv, CameraConfig sensorConfig) {
  gst_rtsp_server_thread = std::thread(
      std::bind(&Camera::run_rtsp_stream, this, argc, argv, sensorConfig));
  gst_rtsp_server_thread.detach();
}

void Camera::run_rtsp_stream(int argc, char **argv, CameraConfig sensorConfig) {
  GMainLoop *loop;
  GstRTSPServer *server;
  GstRTSPMountPoints *mounts;
  GstRTSPMediaFactory *factory;

  gst_init(&argc, &argv);

  // create main loop to spin the server
  loop = g_main_loop_new(NULL, false);

  // create new server
  server = gst_rtsp_server_new();
  g_object_set(server, "service", sensorConfig.port.c_str(), NULL);

  // get mount points to map uri to media factory
  mounts = gst_rtsp_server_get_mount_points(server);

  // media factory to create pipeline, enables rtcp
  factory = gst_rtsp_media_factory_new();

  // apply the pertinent pipeline
  gst_rtsp_media_factory_set_launch(factory, sensorConfig.pipeline.c_str());

  gst_rtsp_media_factory_set_shared(factory, true);
  gst_rtsp_media_factory_set_enable_rtcp(factory, true);

  // attach this factory to the mount point
  gst_rtsp_mount_points_add_factory(mounts, "/video", factory);

  // dont need the pointer anymore
  g_object_unref(mounts);

  // attach server to default context
  gst_rtsp_server_attach(server, NULL);

  // serve that server
  std::string msg =
      "Now serving RTSP video stream on 127.0.0.1:" + sensorConfig.port +
      "/video";
  RCLCPP_INFO(rclcpp::get_logger("rtsp_streamer"), msg.c_str());
  g_main_loop_run(loop);
}

CameraManager *CameraManager::getInstance(int argc, char **argv) {
  static CameraManager instance{argc, argv};
  return &instance;
}

CameraManager::CameraManager(int argc, char **argv) : argc(argc), argv(argv) {}

bool CameraManager::addCamera(CameraConfig sensorConfig) {
  if (cameras.count(sensorConfig.id) > 0) {
    RCLCPP_WARN(rclcpp::get_logger("rtsp_streamer"),
                "Attempted to overwrite sensor, ignoring");
    return false;
  }

  cameras[sensorConfig.id] = Camera(argc, argv, sensorConfig);
  return true;
}

CameraNode::CameraNode(int argc, char **argv) : Node("rov_cameras") {
  manager = CameraManager::getInstance(argc, argv);
  this->declare_parameter("cameras", std::vector<std::string>());

  std::vector<std::string> camera_ids =
      get_parameter("cameras").as_string_array();
  if (camera_ids.size() == 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("rtsp_streamer"),
        "No cameras found, perhaps you forgot to add the params.yaml file?");
  }
  for (auto &name : camera_ids) {
    this->declare_parameter(name + ".port", "");
    this->declare_parameter(name + ".pipeline", "");

    CameraConfig config;
    config.id = name;
    config.port = this->get_parameter(name + ".port").value_to_string();
    config.pipeline = this->get_parameter(name + ".pipeline").as_string();
    if (config.port == "" || config.pipeline == "") {
      RCLCPP_ERROR(rclcpp::get_logger("rtsp_streamer"),
                   "Missing or empty parameters for camera '%s'", name.c_str());
      continue;
    }
    if (!manager->addCamera(config)) {
      RCLCPP_ERROR(rclcpp::get_logger("rtsp_streamer"),
                   "Failed to add camera %s", name.c_str());
    }
  }
}