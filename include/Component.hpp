#ifndef COMPONENT_HPP
#define COMPONENT_HPP

class Component {
public:
    virtual void Update() = 0;
    virtual void AutoUpdate() = 0;
    virtual void Stop() = 0;
};

#endif // COMPONENT_HPP