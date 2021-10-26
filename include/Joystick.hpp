#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <string>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <cassert>
#include <unordered_map>
#include <vector>

#define JS_EVENT_BUTTON 0x01 // button pressed/released
#define JS_EVENT_AXIS   0x02 // joystick moved
#define JS_EVENT_INIT   0x80 // initial state of device

namespace controller {

// Encapsulates all data relevant to a sampled joystick event.
class JoystickEvent
{
public:
    // Minimum value of axes range
    static const short MIN_AXES_VALUE = -32768;

    // Maximum value of axes range
    static const short MAX_AXES_VALUE = 32767;
    
    // The timestamp of the event, in milliseconds.
    unsigned int time;
    
    /**
     * The value associated with this joystick event.
     * For buttons this will be either 1 (down) or 0 (up).
     * For axes, this will range between MIN_AXES_VALUE and MAX_AXES_VALUE.
     */
    short value;
    
    // The event type.
    unsigned char type;
    
    // The axis/button number.
    unsigned char number;

    // Returns true if this event is the result of a button press.
    bool isButton()
    {
        return (type & JS_EVENT_BUTTON) != 0;
    }

    // Returns true if this event is the result of an axis movement.
    bool isAxis()
    {
        return (type & JS_EVENT_AXIS) != 0;
    }
    
    // Returns true if this event is part of the initial setup events
    bool isInitialState()
    {
        return (type & JS_EVENT_INIT) != 0;
    }

    // allow ostream access to member variables so cout << event works
    friend std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);
};

// declare ostream << to allow std::cout << event
std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);

struct Button {
    std::string name;
    unsigned char number;
    short value;
    bool pressedSinceLastCheck = false;
    bool releasedSinceLastCheck = false;
    unsigned int time;

    Button(std::string name) {this->name = name;};
};

struct Axis {
    std::string name;
    unsigned char number;
    short value;
    unsigned int time;

    Axis(std::string name) {this->name = name;};
};

// Wrapper around a map of Button presses to allow smart indexing (READ ONLY)
class ButtonPresses {
public:
    // returns true if button at number was pressed
    bool operator[](unsigned char number);

    // returns true if button at name was pressed
    bool operator[](std::string name);

    friend std::ostream& operator<<(std::ostream& os, const ButtonPresses& presses);
private:
    std::unordered_map<unsigned char, Button> m_byNumber = {};
    std::unordered_map<std::string, Button> m_byName = {};
friend class JSDescriptor;
};

// allow std::cout << ButtonPresses
std::ostream& operator<<(std::ostream& os, const ButtonPresses& presses);

// fancy map that maps names of buttons to button objects
class JSDescriptor {
public:
    std::unordered_map<std::string, std::shared_ptr<Button>> buttonsByString;
    std::unordered_map<std::string, std::shared_ptr<Axis>> axesByString; 
    std::unordered_map<unsigned char, std::shared_ptr<Button>> buttonsByNumber;
    std::unordered_map<unsigned char, std::shared_ptr<Axis>> axesByNumber; 

    JSDescriptor();

    ButtonPresses checkAndGetPresses();
    ButtonPresses getPresses() {return this->presses;};
    ButtonPresses getReleases() {return this->releases;};
protected:
    ButtonPresses presses;
    ButtonPresses releases;
friend class Joystick;
};

// Represents a Logitech Extreme 3D Pro joystick
class Joystick
{
public:
    ~Joystick();

    /*
    * networking=false, opens joystick at /dev/input/js0
    * networking=true, opens tcp-server for joystick events
    */
    Joystick(bool networking=false);

    // opens joystick at /dev/input/js<number>
    Joystick(int joystickNumber);

    // opens joystick at /devicePath/
    Joystick(std::string devicePath);

    // do not allow copying
    Joystick(Joystick const&) = delete;

    // Joystick objects can be moved
    Joystick(Joystick &&) = default;

    // Returns true if the joystick was found and may be used, otherwise false.
    bool isFound();

    // Updates pressed buttons. Call only once per main loop iteration!
    void update();

    ButtonPresses getPresses();
    ButtonPresses getReleases();
private:
    // thread to manage polling controller
    class PollingThread {
    public:
        PollingThread() = default;
        PollingThread(Joystick& js);
        void stop();
    private:
        bool m_isRunning = false;
        std::thread t;
        void poll(bool& running, Joystick& js);
    };

    // internal descriptor of the joystick's buttons and axes
    JSDescriptor m_js;

    // polling thread
    PollingThread t;

    // file descriptor for the appropriate /dev/input/js event
    int _fd;

    // opens the correct /dev/input/js event
    void openPath(std::string devicePath);

    // internal methods for updating button/axis events called from polling thread
    void button_pressed(unsigned char number, unsigned int time);
    void button_released(unsigned char number, unsigned int time);
    void axis_updated(const JoystickEvent& ev);

    /**
     * Attempts to populate the provided JoystickEvent instance with data
     * from the joystick. Returns true if data is available and was populated, otherwise false.
     */
    bool sample(JoystickEvent* event);
};

} // namespace controller

#endif //JOYSTICK_HPP