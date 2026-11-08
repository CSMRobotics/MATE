#ifndef SUB_CONTROLS_H
#define SUB_CONTROLS_H

/**
 * @brief Struct of Submarine controls
 * 
 */
struct SubControls {
    float velocityX;    // arbitrary units
    float velocityY;    // arbitrary units
    float velocityZ;    // m/s

    float omegaX;   // Roll (radians/s)
    float omegaY;   // Pitch
    float omegaZ;   // Yaw
};

#endif // SUB_CONTROLS_H