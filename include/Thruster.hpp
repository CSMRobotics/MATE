#ifndef THRUSTER_HPP
#define THRUSTER_HPP

#include "Component.hpp"

class Thruster : public Component {
    public:
        Thruster();
        ~Thruster();

        void Update();
        void AutoUpdate();
        void Stop();

        void AddThrust(float output, int axis);
    protected:

    private:
        double thrusterPos[3];
        double thrusterAngle[4];
};

#endif // THRUSTER_HPP