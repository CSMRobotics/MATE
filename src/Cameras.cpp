#include <include/Cameras.hpp>

Cameras::Cameras() {
    //initialize all 4 cameras
    for(int i = 0; i < 4; i++){
        cameras[i] = cv::VideoCapture(i);
    }
}

Cameras::~Cameras() {

}

void Cameras::Update() {
    //TODO: decide on what type of buffer we want to pass on constructor
    /*
    for(int i = 0; i < 4; i++){
        if((cameras[i] != NULL) && (cameras[i].isOpened)){
            cameras[i] >> some_buffer
        }
    }
    */
}

void Cameras::AutoUpdate() {

}

void Cameras::Stop() {

}