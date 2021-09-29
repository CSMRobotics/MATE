#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "Component.hpp"

class Drive : public Component {
public:
    Drive();
    ~Drive();

    void Update();
    void AutoUpdate();
    void Stop();
protected:

private:

};

#endif // DRIVE_HPP