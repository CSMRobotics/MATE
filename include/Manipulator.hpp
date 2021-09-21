#include "Component.hpp"
#include <iostream>

// Create basic class definition responsible for moving the robot
class Manipulator : public Component{
public:
    Manipulator();
    ~Manipulator();

    void Update();
    void AutoUpdate();
    void Stop();

    void toggleChicken();

protected:

private:
    //elbow_servo = MANIP_ELBOW_SERVO
    //elbow_servo2 = MANIP_ELBOW_SERVO_2
    //level_servo = MANIP_LEVEL_SERVO
    //wrist_servo = MANIP_WRIST_SERVO
    //clamp_servo = MANIP_CLAMP_SERVO

    bool isChicken = false;
    bool isClamping = false;

    float elbow_angle = 90;
    float elbow_angle_old = 90;
    float wrist_angle = 90;
    float wrist_angle_old = 90;
    float level_angle = 90;
    float level_angle_old = 90;

    float elbow_tune = 11;
    float elbow_tune2 = 12;
    float level_tune = 0;
    float wrist_tune = 15;

    float x_velocity = 0;
    float y_velocity = 0;

    float const VELOCITY_SCALING_FACTOR = .1f;
    float const DELTA_VELOCITY_IGNORE = .1f;
    float const ELBOW_ANGLE_MAX = 180;
    float const ELBOW_ANGLE_MIN = 0;
    float const LEVEL_ANGLE_MAX = 180;
    float const LEVEL_ANGLE_MIN = 0;
    float const WRIST_ANGLE_MAX = 180;
    float const WRIST_ANGLE_MIN = 0;
};