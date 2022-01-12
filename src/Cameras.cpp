#include "Cameras.hpp"

Cameras::Cameras() {
    cameras[0] = cv::VideoCapture(0);
    cameras[1] = cv::VideoCapture(1);
    cameras[2] = cv::VideoCapture(2);
    cameras[3] = cv::VideoCapture(3);
}

void Cameras::Update() {
    std::lock_guard<std::mutex> lock(m_mutex);
    for(int i = 0; i < 4; i++) {
        if(cameras[i].isOpened()) {
            cameras[i].read(frames[i]);
            cv::cvtColor(frames[i], frames[i], cv::COLOR_BGR2RGBA);
        }
    }
}

void Cameras::AutoUpdate() {
    Update();
}

void Cameras::Stop() {
    
}