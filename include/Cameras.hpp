#ifndef CAMERAS_HPP
#define CAMERAS_HPP

#include <chrono>
#include <array>
#include <thread>

#include "opencv2/opencv.hpp"

#include "TCPClientServer.hpp"

class Cameras {

public:
    Cameras();

    void stop();
private:
    void poll(std::reference_wrapper<bool> shouldBeRunning, TCP_Client*);
    bool m_shouldBeRunning = false;
    std::thread m_cameraPollingThread;
};

#endif // CAMERAS_HPP