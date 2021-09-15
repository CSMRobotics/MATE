#include <include/components/Manipulator.hpp>

Manipulator::Manipulator() {
    // TODO: set angles to 0 upon creation
}

Manipulator::~Manipulator() {
    
}

void Manipulator::Update() {
    // TODO: implement game controller updating x/y velocity
    // TODO: update buttons

    if(isChicken){
        elbow_angle = elbow_angle_old + y_velocity;
        level_angle = level_angle_old - y_velocity;
        wrist_angle = wrist_angle_old + x_velocity;
    }
    else {
        level_angle = level_angle_old + y_velocity;
        wrist_angle = wrist_angle_old + x_velocity;
    }

    // keep angles from over/under-shooting
    if(elbow_angle > ELBOW_ANGLE_MAX)
        elbow_angle = ELBOW_ANGLE_MAX;
    else if(elbow_angle < ELBOW_ANGLE_MIN)
        elbow_angle = ELBOW_ANGLE_MIN;
    if(level_angle > LEVEL_ANGLE_MAX)
        level_angle = LEVEL_ANGLE_MAX;
    else if(level_angle < LEVEL_ANGLE_MIN)
        level_angle = LEVEL_ANGLE_MIN;
    if(wrist_angle > WRIST_ANGLE_MAX)
        wrist_angle = WRIST_ANGLE_MAX;
    else if(wrist_angle < WRIST_ANGLE_MIN)
        wrist_angle = WRIST_ANGLE_MIN;

    // update positions
    // always write wrist
    // wrist_servo.angle = wrist_angle + wrist_tune;

    //determines updates based on chicken protocol
    if(isChicken) {
        // elbow_servo.angle = elbow_angle + elbow_tune;
        // elbow_servo2.angle = 180 - elbow_angle + elbow_tune2;
        // level_servo.angle = level_angle + level_tune;
    }
    else {
        // level_servo.angle = level_angle + level_tune;
    }

    // update vars
    elbow_angle_old = elbow_angle;
    level_angle_old = level_angle;
    wrist_angle_old = wrist_angle;

    // TODO: reimplement this to prevent unnecessary torque from being applied to gripped object (overheating possible issue???)
    if(isClamping) {
        // clamp_servo.angle = 130;
    }
    else {
        // clamp_servo.angle = 85;
    }
}

void Manipulator::AutoUpdate() {

}

void Manipulator::Stop() {
    // TODO: set duty cycles to 0
}

void Manipulator::toggleChicken() {
    this->isChicken = !this->isChicken;
}