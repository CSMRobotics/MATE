#ifndef CAMERAS_HPP
#define CAMERAS_HPP

#include <Component.hpp>
#include <array>
#include <opencv4/opencv2/opencv.hpp>

class Cameras : public Component{

public:
    Cameras();
    ~Cameras();

    void Update();
    void AutoUpdate();
    void Stop();

private:
    std::array<cv::VideoCapture, 4> cameras;
    
};

#endif // CAMERAS_HPP