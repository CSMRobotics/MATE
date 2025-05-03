#ifndef CAMERAS_HPP
#define CAMERAS_HPP

#include "gst/gst.h"

#include <cstdint>
#include <thread>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>

struct CameraConfig {
  std::string id;
  std::string port;
  std::string pipeline;
};

class Camera {
public:
  Camera() = default;
  Camera(int argc, char **argv, CameraConfig sensorConfig);

private:
  void run_rtsp_stream(int argc, char **argv, CameraConfig sensorConfig);

  std::thread gst_rtsp_server_thread;
};

class CameraManager {
public:
  static CameraManager *getInstance(int argc, char **argv);

  bool addCamera(CameraConfig sensorConfig);

private:
  CameraManager(int argc, char **argv);

  int argc;
  char **argv;

  std::unordered_map<std::string, Camera> cameras;
};

class CameraNode : public rclcpp::Node {
public:
  CameraNode(int argc, char **argv);

private:
  CameraManager *manager;
};

#endif