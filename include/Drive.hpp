#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "Component.hpp"
#include "Thruster.hpp"

#include <chrono>

class Drive : public Component {
public:
    Drive();
    ~Drive();

    void Update();
    void AutoUpdate();
    void Stop();
protected:

private:
    uint64_t millis() {
        uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
        return ms;
    }
    uint64_t timeInitialMillis, timePreviousMillis, timeCurrentMillis;
};

#endif // DRIVE_HPP