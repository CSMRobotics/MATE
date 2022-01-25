#include "Cameras.hpp"

Cameras::Cameras() {

}

void Cameras::stop() {
    m_shouldBeRunning = false;
}

void Cameras::poll(std::reference_wrapper<bool> shouldBeRunning, TCP_Client* client) {
    cv::VideoCapture cameraTL = cv::VideoCapture(0);
    cv::VideoCapture cameraTR = cv::VideoCapture(1);
    cv::VideoCapture cameraBL = cv::VideoCapture(2);
    cv::VideoCapture cameraBR = cv::VideoCapture(3);
    
    cv::Mat TL = cv::Mat(240, 320, CV_8UC3);
    cv::Mat TR = cv::Mat(240, 320, CV_8UC3);
    cv::Mat BL = cv::Mat(240, 320, CV_8UC3);
    cv::Mat BR = cv::Mat(240, 320, CV_8UC3);
    cv::Mat HALFFRAMETOP = cv::Mat(240, 640, CV_8UC3);
    cv::Mat HALFFRAMEBOT = cv::Mat(240, 640, CV_8UC3);
    cv::Mat FRAME = cv::Mat(480, 640, CV_8UC3);

    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::system_clock> finish;
    while(true) {
        start = std::chrono::high_resolution_clock::now();
        // read the frames from camera
        cameraTL.read(TL);
        cameraTR.read(TR);
        cameraBL.read(BL);
        cameraBR.read(BR);
        // concatenate into a single image
        cv::hconcat(TL, TR, HALFFRAMETOP);
        cv::hconcat(BL, BR, HALFFRAMEBOT);
        cv::vconcat(HALFFRAMETOP, HALFFRAMEBOT, FRAME);
        // copy frame to array in heap
        cv::Mat* toSend = new cv::Mat(480, 640, CV_8UC3);
        FRAME.copyTo(*toSend);
        // send frame to Driver Station
        client->sendMessage(toSend);
        finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> timeItTook = (start-finish);
        std::cout << timeItTook.count() << std::endl;
    }
}