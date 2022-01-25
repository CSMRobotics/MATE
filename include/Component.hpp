#ifndef COMPONENT_HPP
#define COMPONENT_HPP

class Component {
public:
    // Called once per robot update
    virtual void Update() = 0;
    
    // Called once per robot update when in full autonomous mode
    virtual void AutoUpdate() = 0;

    // Called to stop component and put the component in an idle mode (e.g. in event of E-Stop)
    virtual void Stop() = 0;
};

#endif // COMPONENT_HPP