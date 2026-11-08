#ifndef SUB_CONTROLS_H
#define SUB_CONTROLS_H

/**
 * @brief Struct of Submarine controls
 *      All controls are normalized and unitless
 */
struct SubControls {
    float velocityX;    // +Forward
    float velocityY;    // +Right
    float velocityZ;    // +Down

    float omegaX;   // Roll, +Clockwise looking down x axis, from back to front
    float omegaY;   // Pitch, +Pull up
    float omegaZ;   // Yaw, +Turn right

    bool mode_switch;
    bool estop;
};

#endif // SUB_CONTROLS_H