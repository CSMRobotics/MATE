#include "Joystick.hpp"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>

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
    button->pressedSinceLastCheck = true;
}

void Joystick::button_released(unsigned char number, unsigned int time) {
    Button* button = m_js.buttonsByNumber.at(number).get();

    button->value = 0;
    button->time = 0;
    button->releasedSinceLastCheck = true;
}

void Joystick::axis_updated(const JoystickEvent& ev) {
    Axis* axis = m_js.axesByNumber.at(ev.number).get();

    axis->value = ev.value;
    axis->time = ev.time;
}

void Joystick::update_presses() {
    m_presses.clear();

    for(std::pair<unsigned char, std::shared_ptr<Button>> mapButton : m_js.buttonsByNumber) {
        Button* bt = mapButton.second.get();

        if(bt->pressedSinceLastCheck){
            m_presses.emplace(mapButton.second);
        }
    }
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
