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