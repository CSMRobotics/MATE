#include "Joystick.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
#include "CSMUtil.hpp"

using namespace controller;

Joystick::Joystick(bool networking) {
    if(networking) {
        // TODO: IMPLEMENT
    }
    else {
        openPath("/dev/input/js0");
    }
}

Joystick::Joystick(int joystickNumber) {
    std::stringstream sstm;
    sstm << "/dev/input/js" << joystickNumber;
    openPath(sstm.str());
}

Joystick::Joystick(std::string devicePath) {
    openPath(devicePath);
}

void Joystick::openPath(std::string devicePath)
{
    bool blocking = false;
    // Open the device using either blocking or non-blocking
    _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
}

bool Joystick::sample(JoystickEvent* ev) {
    int bytes = read(_fd, ev, sizeof(*ev));

    if(bytes == -1)
        return false;

    // if this condition is not met, controller is out of sync and this
    // joystick instance is likely unusable
    return bytes == sizeof(*ev);
}

void Joystick::button_pressed(unsigned char number, unsigned int time) {
    Button* button = m_js.buttonsByNumber.at(number).get();

    button->value = 1;
    button->time = time;
    button->number = number;
    button->pressedSinceLastCheck = true;
}

void Joystick::button_released(unsigned char number, unsigned int time) {
    Button* button = m_js.buttonsByNumber.at(number).get();

    button->value = 0;
    button->time = 0;
    button->number = number;
    button->releasedSinceLastCheck = true;
}

void Joystick::axis_updated(const JoystickEvent& ev) {
    Axis* axis = m_js.axesByNumber.at(ev.number).get();

    axis->value = ev.value;
    axis->time = ev.time;
    axis->number = ev.number;
}

void Joystick::update() {
    m_js.checkAndGetPresses();
}

bool Joystick::isFound()
{
    return _fd >= 0;
}

Joystick::~Joystick()
{
    close(_fd);
}

std::ostream& controller::operator<<(std::ostream& os, const JoystickEvent& e)
{
    os << "type=" << static_cast<int>(e.type)
       << " number=" << static_cast<int>(e.number)
       << " value=" << static_cast<int>(e.value);
    return os;
}

Joystick::PollingThread::PollingThread(Joystick& js) {
    assert(("Joystick not found, unable to start thread.", js.isFound()));
    this->m_isRunning = true;
    t = std::thread(&PollingThread::poll, this, std::ref(this->m_isRunning), std::ref(js));
}

void Joystick::PollingThread::poll(bool& running, Joystick& js) {
    JoystickEvent ev;
    while(running) {
        // ensure we wait at least 1 ms before polling again
        usleep(1000);

        // sample the joystick for an event
        if(js.sample(&ev)) {
            // update the joystick accordingly
            if(ev.isInitialState()) {
                // TODO: do we even need to care???
            }
            if(ev.isButton()) {
                if(ev.value) {
                    js.button_pressed(ev.number, ev.time);
                } else {
                    js.button_released(ev.number, ev.time);
                }
            } else if(ev.isAxis()) {
                js.axis_updated(ev);
            }
        }
    }
}

void Joystick::PollingThread::stop() {
    m_isRunning = false;
    t.join();
}

JSDescriptor::JSDescriptor() {
    std::shared_ptr<Button> BUTTON_0 = std::make_shared<Button>("BUTTON_0");
    buttonsByString.emplace(std::make_pair(BUTTON_0.get()->name,BUTTON_0));
    buttonsByNumber.emplace(std::make_pair(0, BUTTON_0));

    std::shared_ptr<Button> BUTTON_1 = std::make_shared<Button>("Button_1");
    buttonsByString.emplace(std::make_pair(BUTTON_1.get()->name, BUTTON_1));
    buttonsByNumber.emplace(std::make_pair(1, BUTTON_1));

    std::shared_ptr<Button> BUTTON_2 = std::make_shared<Button>("BUTTON_2");
    buttonsByString.emplace(std::make_pair(BUTTON_2.get()->name, BUTTON_2));
    buttonsByNumber.emplace(std::make_pair(2, BUTTON_2));

    std::shared_ptr<Button> BUTTON_3 = std::make_shared<Button>("BUTTON_3");
    buttonsByString.emplace(std::make_pair(BUTTON_3.get()->name, BUTTON_3));
    buttonsByNumber.emplace(std::make_pair(3, BUTTON_3));

    std::shared_ptr<Button> BUTTON_4 = std::make_shared<Button>("BUTTON_4");
    buttonsByString.emplace(std::make_pair(BUTTON_4.get()->name, BUTTON_4));
    buttonsByNumber.emplace(std::make_pair(4, BUTTON_4));

    std::shared_ptr<Button> BUTTON_5 = std::make_shared<Button>("BUTTON_5");
    buttonsByString.emplace(std::make_pair(BUTTON_5.get()->name, BUTTON_5));
    buttonsByNumber.emplace(std::make_pair(5, BUTTON_5));

    std::shared_ptr<Button> BUTTON_6 = std::make_shared<Button>("BUTTON_6");
    buttonsByString.emplace(std::make_pair(BUTTON_6.get()->name, BUTTON_6));
    buttonsByNumber.emplace(std::make_pair(6, BUTTON_6));

    std::shared_ptr<Button> BUTTON_7 = std::make_shared<Button>("BUTTON_7");
    buttonsByString.emplace(std::make_pair(BUTTON_7.get()->name, BUTTON_7));
    buttonsByNumber.emplace(std::make_pair(7, BUTTON_7));

    std::shared_ptr<Button> BUTTON_8 = std::make_shared<Button>("BUTTON_8");
    buttonsByString.emplace(std::make_pair(BUTTON_8.get()->name, BUTTON_8));
    buttonsByNumber.emplace(std::make_pair(8, BUTTON_8));

    std::shared_ptr<Button> BUTTON_9 = std::make_shared<Button>("BUTTON_9");
    buttonsByString.emplace(std::make_pair(BUTTON_9.get()->name, BUTTON_9));
    buttonsByNumber.emplace(std::make_pair(9, BUTTON_9));

    std::shared_ptr<Button> BUTTON_10 = std::make_shared<Button>("BUTTON_10");
    buttonsByString.emplace(std::make_pair(BUTTON_10.get()->name, BUTTON_10));
    buttonsByNumber.emplace(std::make_pair(10, BUTTON_10));

    std::shared_ptr<Button> BUTTON_11 = std::make_shared<Button>("BUTTON_11");
    buttonsByString.emplace(std::make_pair(BUTTON_11.get()->name, BUTTON_11));
    buttonsByNumber.emplace(std::make_pair(11, BUTTON_11));


    std::shared_ptr<Axis> AXIS_PITCH = std::make_shared<Axis>("AXIS_PITCH");
    axesByString.emplace(std::make_pair(AXIS_PITCH.get()->name, AXIS_PITCH));
    axesByNumber.emplace(std::make_pair(0, AXIS_PITCH));

    std::shared_ptr<Axis> AXIS_ROLL = std::make_shared<Axis>("AXIS_ROLL");
    axesByString.emplace(std::make_pair(AXIS_ROLL.get()->name, AXIS_ROLL));
    axesByNumber.emplace(std::make_pair(1, AXIS_ROLL));

    std::shared_ptr<Axis> AXIS_YAW = std::make_shared<Axis>("AXIS_YAW");
    axesByString.emplace(std::make_pair(AXIS_YAW.get()->name, AXIS_YAW));
    axesByNumber.emplace(std::make_pair(2, AXIS_YAW));

    std::shared_ptr<Axis> HAT_X = std::make_shared<Axis>("HAT_X");
    axesByString.emplace(std::make_pair(HAT_X.get()->name, HAT_X));
    axesByNumber.emplace(std::make_pair(3, HAT_X));

    std::shared_ptr<Axis> HAT_Y = std::make_shared<Axis>("HAT_Y");
    axesByString.emplace(std::make_pair(HAT_Y.get()->name, HAT_Y));
    axesByNumber.emplace(std::make_pair(4, HAT_Y));
}

ButtonPresses JSDescriptor::checkAndGetPresses() {
    this->presses = ButtonPresses();
    this->releases = ButtonPresses();
    
    for(auto it : buttonsByNumber) {
        if(it.second.get()->pressedSinceLastCheck) {
            presses.m_byNumber.emplace(std::make_pair(it.first, *it.second.get()));
            presses.m_byName.emplace(std::make_pair(it.second.get()->name, *it.second.get()));
            it.second.get()->pressedSinceLastCheck = false;
        }
        if(it.second.get()->releasedSinceLastCheck) {
            releases.m_byNumber.emplace(std::make_pair(it.first, *it.second.get()));
            releases.m_byName.emplace(std::make_pair(it.second.get()->name, *it.second.get()));
            it.second.get()->releasedSinceLastCheck = false;
        }
    }

    return this->presses;
}

float JSDescriptor::getAxisValue(unsigned char number) {
    return axesByNumber.at(number).get()->value;
}

float JSDescriptor::getAxisValue(std::string name) {
    return axesByString.at(name).get()->value;
}

Axis* JSDescriptor::getAxis(unsigned char number) {
    return axesByNumber.at(number).get();
}

Axis* JSDescriptor::getAxis(std::string name) {
    return axesByString.at(name).get();
}

float Axis::normalizeAxis() {
    return csmutil::map<float, float, float>(value, -32768, 32767, -1, 1);
}

bool ButtonPresses::operator[](unsigned char number) {
    try {
        m_byNumber.at(number);
        return true;
    } catch (const std::out_of_range& e) {
        return false;
    }

    return false;
}

bool ButtonPresses::operator[](std::string name) {
    try {
        m_byName.at(name);
        return true;
    } catch (const std::out_of_range& e) {
        return false;
    }

    return false;
}

std::ostream& controller::operator<<(std::ostream& os, const ButtonPresses& presses) {
    os << "Presses/Releases:[";
    for(auto mapObj : presses.m_byName) {
        os << "{Name=" << mapObj.first << "},";
    }
    os << "]";
    return os;
}

ButtonPresses Joystick::getPresses() {
    return m_js.getPresses();
}

ButtonPresses Joystick::getReleases() {
    return m_js.getReleases();
}

