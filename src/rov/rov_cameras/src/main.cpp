#if DEBUG_OUTPUT
    #include <sstream>
#endif

#include <set>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>

#include "rov_cameras/enumerate_cameras.hpp"
#include "rov_cameras/h264_camera.hpp"
#include "rov_cameras/mjpeg_camera.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::unordered_map<std::string, Camera_Metadata> camera_metadata;
    enumerateCameras(camera_metadata);

    // sets simplify my life when parsing
    std::set<std::string> mjpeg_cameras;
    std::set<std::string> mjpeg_devices;
    std::set<std::string> h264_cameras;
    std::set<std::string> h264_devices;
    
#if DEBUG_OUTPUT
    std::stringstream ss;
#endif
    // for each camera enumerated
    for(auto c : camera_metadata) {
#if DEBUG_OUTPUT
        ss << "Devices for camera" << c.first.c_str() << '\n';
        // printf("Devices for camera %s\n", c.first.c_str());
#endif
        // for each v4l2 device associated with each camera
        for(auto d : c.second.device_names) {
#if DEBUG_OUTPUT
            ss << '\t' << d.c_str() << '\t';
            // printf("\t%s\n", d.c_str());
#endif
            try {
                // for each format associated with each v4l2 device
                for(auto f : c.second.formats.at(d)) {
#if DEBUG_OUTPUT
                    ss << "\t\t" << f.c_str() << '\n';
                    // printf("\t\t%s\n", f.c_str());
#endif
                    if(f == "Motion-JPEG") {
                        mjpeg_devices.emplace(d);
                        mjpeg_cameras.emplace(c.first);
                    }

                    if(f == "H.264") {
                        h264_devices.emplace(d);
                        h264_cameras.emplace(c.first);
                    }
                }
            } catch ([[maybe_unused]] std::out_of_range& e) {}
        }
    }
#if DEBUG_OUTPUT
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s\n", ss.str());
#endif

    // calculate the intersection of cameras that support mjpeg and h264
    // yeah i do set theory (⌐□_□)
    std::set<std::string> camera_intersection;
    std::set_intersection(mjpeg_cameras.begin(), mjpeg_cameras.end(), h264_cameras.begin(), h264_cameras.end(), std::inserter(camera_intersection, camera_intersection.begin()));

    // prefer h264
    for(auto cam : camera_intersection) {
        mjpeg_cameras.erase(cam);
        for(auto dev : mjpeg_devices) {
            auto it = std::find(camera_metadata.at(cam).device_names.begin(), camera_metadata.at(cam).device_names.end(), dev);
            if(it != camera_metadata.at(cam).device_names.end()) {
                mjpeg_devices.erase(*it);
            }
        }
    }

#if DEBUG_OUTPUT
    printf("\n\nMJPEG Cameras\n");
    for(auto cam : mjpeg_cameras) {
        printf("\t%s\n", cam.c_str());
    }

    printf("MJPEG Devices\n");
    for(auto dev : mjpeg_devices) {
        printf("\t%s\n", dev.c_str());
    }

    printf("H264 Cameras\n");
    for(auto cam : h264_cameras) {
        printf("\t%s\n", cam.c_str());
    }

    printf("H264 Devices\n");
    for(auto dev : h264_devices) {
        printf("\t%s\n", dev.c_str());
    }
#endif

    // add all cameras to multithreaded executor
    rclcpp::executors::MultiThreadedExecutor exec;
    std::vector<rclcpp::Node::SharedPtr> mjpeg_nodes;
    std::vector<rclcpp::Node::SharedPtr> h264_nodes;
    
    int camera_idx = 0;
    for(auto device : mjpeg_devices) {
        mjpeg_nodes.emplace_back(std::make_shared<MJPEG_Camera>(device, camera_idx));
        exec.add_node(mjpeg_nodes.back());
        camera_idx++;
    }
    for(auto device : h264_devices) {
        h264_nodes.emplace_back(std::make_shared<H264_Camera>(device, camera_idx));
        exec.add_node(h264_nodes.back());
        camera_idx++;
    }

    exec.spin();

    rclcpp::shutdown();
    return 0;
}