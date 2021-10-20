#ifndef JOYSTICK_HPP
#define JOYSTICK_HPP

#include <string>
#include <iostream>

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

// Represents a joystick
class Joystick
{
private:
  void openPath(std::string devicePath);
  
  int _fd;
  
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
  
  /**
   * Attempts to populate the provided JoystickEvent instance with data
   * from the joystick. Returns true if data is available and was populated, otherwise false.
   */
  bool sample(JoystickEvent* event);
};

} // namespace controller

#endif //JOYSTICK_HPP