#ifndef HORRIBLE_ROS_PATH_HEADER_INCLUDED
#define HORRIBLE_ROS_PATH_HEADER_INCLUDED

#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>
#define PREFIX std::filesystem::path(ament_index_cpp::get_package_share_directory("driverstation_gui"))

namespace driverstation::horrible_ros_path {

    std::string prepend_prefix(char c_str[]) {
        return std::string((PREFIX / c_str).c_str());
    }

    std::string prepend_prefix(std::string& str) {
        return std::string((PREFIX / str.c_str()).c_str());
    }

}

#endif