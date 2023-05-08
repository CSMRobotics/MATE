#ifndef ENUMERATE_CAMERAS_INCLUDED_HPP
#define ENUMERATE_CAMERAS_INCLUDED_HPP

#include <unordered_map>
#include <vector>

extern "C" {
#include <stdio.h>
#include <string.h>
#include <libudev.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
}

#define DELIM " -> "
#define REMOVE "../../"

struct Camera_Metadata {
    std::vector<std::string> device_names;
    std::unordered_map<std::string, std::vector<std::string>> formats;
    v4l2_capability capability; 
};

// enumerate udev devices that match subsystem video4linux and evaluate them for capture formats and v4l2 capabilities
void enumerateCameras(std::unordered_map<std::string, Camera_Metadata>& cameras) {
    struct udev *udev;
    struct udev_enumerate *enumerate;
    struct udev_list_entry *devices, *dev_list_entry;
    struct udev_device *dev;

    udev = udev_new();

    enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "video4linux");
    udev_enumerate_scan_devices(enumerate);
    devices = udev_enumerate_get_list_entry(enumerate);

    udev_list_entry_foreach(dev_list_entry, devices) {
        const char *path;
    
        path = udev_list_entry_get_name(dev_list_entry);
        dev = udev_device_new_from_syspath(udev, path);
        // printf("path: %s\n", path);
        // printf("serial_id: %s\n", udev_device_get_property_value(dev, "ID_PATH"));
        const char* camera_id_path = udev_device_get_property_value(dev, "ID_PATH");
        
        auto it = cameras.find(std::string(camera_id_path));
        if(it != cameras.end()) {
            char* dev_name = strdup(udev_device_get_property_value(dev, "DEVNAME"));
            // printf("Device Name: %s\n", dev_name);
            it->second.device_names.emplace_back(std::string(dev_name));
            free(dev_name);
        } else {
            cameras.emplace(std::make_pair<std::string, Camera_Metadata>(std::string(camera_id_path), Camera_Metadata()));
            char* dev_name = strdup(udev_device_get_property_value(dev, "DEVNAME"));
            // printf("Device Name: %s\n", dev_name);
            cameras.at(std::string(camera_id_path)).device_names.emplace_back(std::string(dev_name));
            free(dev_name);
        }

        int fd = open(cameras.at(std::string(camera_id_path)).device_names.back().c_str(), O_RDWR);

        v4l2_capability capability;
        if(ioctl(fd, VIDIOC_QUERYCAP, &capability) != -1) {
            cameras.at(std::string(camera_id_path)).capability = capability;
            // printf("\tDriver: %s\n\tCard: %s\n\tBus info: %s\n\tCapabilities: %0X\n", capability.driver, capability.card, capability.bus_info, capability.capabilities);
        }

        if(fd != -1) {
            std::string format = "";
            v4l2_fmtdesc fmtdesc;
            memset(&fmtdesc, 0, sizeof(fmtdesc));
            fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            while(ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
                format = format + (char*)fmtdesc.description;
                if(format != "") {
                    cameras.at(std::string(camera_id_path)).formats[cameras.at(std::string(camera_id_path)).device_names.back().c_str()].emplace_back(format);
                    // printf("Format: %s\n", format.c_str());
                    format = "";
                }
                fmtdesc.index++;
            }
        }
        close(fd);

        // printf("subsystem: %s\n", udev_device_get_sysattr_value(dev, "subsystem:"));
        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);
    udev_unref(udev);

}

#endif