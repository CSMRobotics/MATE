#include "Joystick.hpp"

Buttons::Buttons() {
    std::shared_ptr<Button> BUTTON_0 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(0, BUTTON_0));

    std::shared_ptr<Button> BUTTON_1 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(1, BUTTON_1));
    
    std::shared_ptr<Button> BUTTON_2 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(2, BUTTON_2));
    
    std::shared_ptr<Button> BUTTON_3 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(3, BUTTON_3));
    
    std::shared_ptr<Button> BUTTON_4 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(4, BUTTON_4));
    
    std::shared_ptr<Button> BUTTON_5 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(5, BUTTON_5));
    
    std::shared_ptr<Button> BUTTON_6 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(6, BUTTON_6));
    
    std::shared_ptr<Button> BUTTON_7 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(7, BUTTON_7));
    
    std::shared_ptr<Button> BUTTON_8 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(8, BUTTON_8));
    
    std::shared_ptr<Button> BUTTON_9 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(9, BUTTON_9));
    
    std::shared_ptr<Button> BUTTON_10 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(10, BUTTON_10));
    
    std::shared_ptr<Button> BUTTON_11 = std::make_shared<Button>();
    buttons.emplace(std::make_pair(11, BUTTON_11));
    
}

bool Buttons::operator[](unsigned char number) {
    try {
        return buttons.at(number).get()->value == 1;
    } catch (std::out_of_range) {}
    return false;
}

ButtonPresses::ButtonPresses() {
    presses.fill(false);
}

bool ButtonPresses::operator[](unsigned char number) {
    return presses[static_cast<int>(number)];
}

Axes::Axes() {
    std::shared_ptr<Axis> AXIS_0 = std::make_shared<Axis>();
    axes.emplace(std::make_pair(0, AXIS_0));
    
    std::shared_ptr<Axis> AXIS_1 = std::make_shared<Axis>();
    axes.emplace(std::make_pair(1, AXIS_1));

    std::shared_ptr<Axis> AXIS_2 = std::make_shared<Axis>();
    axes.emplace(std::make_pair(2, AXIS_2));

    std::shared_ptr<Axis> AXIS_3 = std::make_shared<Axis>();
    axes.emplace(std::make_pair(3, AXIS_3));
    
    std::shared_ptr<Axis> HAT_0 = std::make_shared<Axis>();
    axes.emplace(std::make_pair(4, HAT_0));
    
    std::shared_ptr<Axis> HAT_1 = std::make_shared<Axis>();
    axes.emplace(std::make_pair(5, HAT_1));
}

float Axes::operator[](unsigned char number) {
    try {
        return axes.at(number).get()->normalize();
    } catch (std::out_of_range) {}
    return 0.0f;
}

Joystick::~Joystick() {
    pollingThread->stop();
}

Joystick::Joystick(int joystickNumber) {
    init();
    std::stringstream ss;
    ss << "/dev/input/js" << joystickNumber;
    pollingThread = new PollingThread(ss.str(), this);
}

Joystick::Joystick(std::string path) {
    init();
    pollingThread = new PollingThread(path, this);
}

void Joystick::init() {
    presses = new ButtonPresses();
    releases = new ButtonPresses();
}

bool Joystick::isFound() {
    return pollingThread->_fd >= 0;
}

void Joystick::updatePresses() {
    presses->presses.fill(false);
    releases->presses.fill(false);

    for(auto pair : buttons.buttons) {
        if(pair.second.get()->pressedSinceLastCheck) {
            presses->presses[static_cast<int>(pair.first)] = true;
            pair.second.get()->pressedSinceLastCheck = false;
        }
        if(pair.second.get()->releasedSinceLastCheck) {
            releases->presses[static_cast<int>(pair.first)] = true;
            pair.second.get()->releasedSinceLastCheck = false;
        }
    }
}

ButtonPresses Joystick::getPresses() {
    return *presses;
}

ButtonPresses Joystick::getReleases() {
    return *releases;
}

// technically this should have a lock but......
Axes Joystick::getAxes() {
    return axes;
}

void Joystick::button_pressed(unsigned char number, unsigned int time) {
    // std::cout << "Button " << static_cast<int>(number) << " pressed" << '\n';
    Button* button = buttons.buttons.at(number).get();
    button->value = 1;
    button->time = time;
    button->pressedSinceLastCheck = true;
}

void Joystick::button_released(unsigned char number, unsigned int time) {
    // std::cout << "Button " << static_cast<int>(number) << " released" << '\n';
    Button* button = buttons.buttons.at(number).get();
    button->value = 0;
    button->time = time;
    button->releasedSinceLastCheck = true;
}

void Joystick::axis_updated(const JoystickEvent& ev) {
    // std::cout << "Axis " << static_cast<int>(ev.number) << " updated" << '\n';
    Axis* axis = axes.axes.at(ev.number).get();
    axis->value = ev.value;
    axis->time = ev.time;
}

std::ostream& operator<<(std::ostream& os, const JoystickEvent& e)
{
    os << "type=" << static_cast<int>(e.type)
       << " number=" << static_cast<int>(e.number)
       << " value=" << static_cast<int>(e.value);
    return os;
}

PollingThread::PollingThread(std::string path, Joystick* joystick) {
    this->_fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
    assert(("Joystick not found", this->_fd >= 0));
    m_isRunning = true;
    thread = std::thread(&PollingThread::poll, this, std::ref(m_isRunning), joystick);
    thread.detach();
}

void PollingThread::poll(std::reference_wrapper<bool> running, Joystick* joystick) {
    JoystickEvent event;
    while(running.get()) {
        usleep(1000);
        if(sample(&event)) {
            if(event.isInitialState()) {
                // do we care?
            }
            if(event.isButton()) {
                if(event.value)
                    joystick->button_pressed(event.number, event.time);
                else
                    joystick->button_released(event.number, event.time);
            } else if(event.isAxis()) {
                joystick->axis_updated(event);
            }
        }
    }
}

bool PollingThread::sample(JoystickEvent* event) {
    int bytes = read(_fd, event, sizeof(*event));

    if(bytes == -1)
        return false;

    return bytes == sizeof(*event);
}

void PollingThread::stop() {
    m_isRunning = false;
    usleep(5000); // wait for thread to *hopefully* stop NOT GUARANTEED
}