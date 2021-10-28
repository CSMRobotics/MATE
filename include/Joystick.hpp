#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <string>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>
#include <memory>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <assert.h>
#include <unordered_map>

#include "CSMUtil.hpp"

#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // joystick moved
#define JS_EVENT_INIT   0x80 // initial state of device

class PollingThread;

// represents a single button on the joystick
struct Button {
    short value;
    unsigned int time;

    bool pressedSinceLastCheck = false;
    bool releasedSinceLastCheck = false;
};

// up to date information on every button's current state
class Buttons {
public:
    Buttons();
    bool operator[](unsigned char number);
private:
    std::unordered_map<unsigned char, std::shared_ptr<Button>> buttons = {};
friend class Joystick;
};

// has a button been pressed since last time we checked?
class ButtonPresses {
public:
    ButtonPresses();
    bool operator[](unsigned char number);
private:
    std::array<bool, 12> presses;
friend class Joystick;
};

// represents a single axis on the joystick
struct Axis {
    static const short MIN_AXES_VALUE = -32768;
    static const short MAX_AXES_VALUE = 32767;

    short value;
    unsigned int time;

    float normalize() {return csmutil::map<float,float,float>(value, MIN_AXES_VALUE, MAX_AXES_VALUE, -1, 1);};
};

// up to date information on every axis' current state
class Axes {
public:
    Axes();
    float operator[](unsigned char number);
private:
    std::unordered_map<unsigned char, std::shared_ptr<Axis>> axes;
friend class Joystick;
};

// represents an evdev event for the joystick
class JoystickEvent
{
public:
    unsigned int time;
    short value;
    unsigned char type;
    unsigned char number;
    bool isButton()
    {
        return (type & JS_EVENT_BUTTON) != 0;
    }
    bool isAxis()
    {
        return (type & JS_EVENT_AXIS) != 0;
    }
    bool isInitialState()
    {
        return (type & JS_EVENT_INIT) != 0;
    }
    friend std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);
};
std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);

// represents a Logitech Extreme 3d Pro joystick
class Joystick
{
public:
    ~Joystick();
    Joystick(int joystickNumber);
    Joystick(std::string path);
    bool isFound();
    void updatePresses();
    ButtonPresses getPresses();
    ButtonPresses getReleases();
    Axes getAxes();
private:
    PollingThread* pollingThread;
    ButtonPresses* presses;
    ButtonPresses* releases;
    Buttons buttons;
    Axes axes;

    void button_pressed(unsigned char number, unsigned int time);
    void button_released(unsigned char number, unsigned int time);
    void axis_updated(const JoystickEvent& ev);

    void init();
friend class PollingThread;
};

class PollingThread {
public:
    PollingThread() = default;
    PollingThread(std::string path, Joystick* joystick);
    void stop();
private:
    int _fd;
    bool m_isRunning = false;
    std::thread thread;
    void openPath(std::string devicePath);
    void poll(std::reference_wrapper<bool> running, Joystick* joystick);
    bool sample(JoystickEvent* event);
friend class Joystick;
};

#endif //JOYSTICK_HPP