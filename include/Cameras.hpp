#ifndef CAMERAS_HPP
#define CAMERAS_HPP

#include <array>
#include <mutex>
#include "opencv2/opencv.hpp"
#include "Component.hpp"

class Cameras : public Component{

public:
    Cameras();

    void Update();
    void AutoUpdate();
    void Stop();

    cv::Mat getFrame(int camera) {return frames[camera];};

private:
    std::array<cv::VideoCapture, 4> cameras;
    std::array<cv::Mat, 4> frames;
    std::mutex m_mutex;
};

#endif // CAMERAS_HPP