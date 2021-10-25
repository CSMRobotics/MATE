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
#include <unordered_set>

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

// declare ostream << to allow cout << event
std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);

struct Button {
    short value;
    bool pressedSinceLastCheck = false;
    bool releasedSinceLastCheck = false;
    unsigned int time;
};

struct Axis {
    short value;
    unsigned int time;
};

struct JSDescriptor {
    std::unordered_map<std::string, std::shared_ptr<Button>> buttonsByString;
    std::unordered_map<unsigned char, std::shared_ptr<Button>> buttonsByNumber;

    std::unordered_map<std::string, std::shared_ptr<Axis>> axesByString;
    std::unordered_map<unsigned char, std::shared_ptr<Axis>> axesByNumber;

    JSDescriptor() {
        std::shared_ptr<Button> BUTTON_0 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_0",BUTTON_0));
        buttonsByNumber.emplace(std::make_pair(0, BUTTON_0));

        std::shared_ptr<Button> BUTTON_1 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_1",BUTTON_1));
        buttonsByNumber.emplace(std::make_pair(1, BUTTON_1));

        std::shared_ptr<Button> BUTTON_2 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_2",BUTTON_2));
        buttonsByNumber.emplace(std::make_pair(2, BUTTON_2));

        std::shared_ptr<Button> BUTTON_3 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_3",BUTTON_3));
        buttonsByNumber.emplace(std::make_pair(3, BUTTON_3));

        std::shared_ptr<Button> BUTTON_4 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_4",BUTTON_4));
        buttonsByNumber.emplace(std::make_pair(4, BUTTON_4));

        std::shared_ptr<Button> BUTTON_5 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_5",BUTTON_5));
        buttonsByNumber.emplace(std::make_pair(5, BUTTON_5));

        std::shared_ptr<Button> BUTTON_6 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_6",BUTTON_6));
        buttonsByNumber.emplace(std::make_pair(6, BUTTON_6));

        std::shared_ptr<Button> BUTTON_7 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_7",BUTTON_7));
        buttonsByNumber.emplace(std::make_pair(7, BUTTON_7));

        std::shared_ptr<Button> BUTTON_8 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_8",BUTTON_8));
        buttonsByNumber.emplace(std::make_pair(8, BUTTON_8));

        std::shared_ptr<Button> BUTTON_9 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_9",BUTTON_9));
        buttonsByNumber.emplace(std::make_pair(9, BUTTON_9));

        std::shared_ptr<Button> BUTTON_10 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_10",BUTTON_10));
        buttonsByNumber.emplace(std::make_pair(10, BUTTON_10));

        std::shared_ptr<Button> BUTTON_11 = std::make_shared<Button>();
        buttonsByString.emplace(std::make_pair("BUTTON_11",BUTTON_11));
        buttonsByNumber.emplace(std::make_pair(11, BUTTON_11));


        std::shared_ptr<Axis> AXIS_PITCH = std::make_shared<Axis>();
        axesByString.emplace(std::make_pair("AXIS_PITCH", AXIS_PITCH));
        axesByNumber.emplace(std::make_pair(0, AXIS_PITCH));

        std::shared_ptr<Axis> AXIS_ROLL = std::make_shared<Axis>();
        axesByString.emplace(std::make_pair("AXIS_ROLL", AXIS_ROLL));
        axesByNumber.emplace(std::make_pair(1, AXIS_ROLL));

        std::shared_ptr<Axis> AXIS_YAW = std::make_shared<Axis>();
        axesByString.emplace(std::make_pair("AXIS_YAW", AXIS_YAW));
        axesByNumber.emplace(std::make_pair(2, AXIS_YAW));

        std::shared_ptr<Axis> HAT_X = std::make_shared<Axis>();
        axesByString.emplace(std::make_pair("HAT_X", HAT_X));
        axesByNumber.emplace(std::make_pair(3, HAT_X));

        std::shared_ptr<Axis> HAT_Y = std::make_shared<Axis>();
        axesByString.emplace(std::make_pair("HAT_Y", HAT_Y));
        axesByNumber.emplace(std::make_pair(4, HAT_Y));
    }

    bool isPressed(Button& button) {
        return button.value == 1;
    }

    unsigned int held(Button& button) {
        return 1;
    }
};

// Represents a Loigitech Extreme 3D Pro joystick
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
    void update_presses();
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

    // interal descriptor of buttons that have been pressed since last update call
    std::unordered_set<std::shared_ptr<Button>> m_presses;

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