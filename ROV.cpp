#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"
#include "Joystick.hpp"
#include "ServoDriver.hpp"
#include "IMU.hpp"

Component* components[2];
Component* activeComponent; // which component should listen to joystick?
TCP_Client* client = new TCP_Client();
TCP_Server* server = new TCP_Server();
ServoDriver* servoDriver = new ServoDriver();
Joystick js = Joystick(server); // create network joystick
// Joystick js = Joystick("/dev/input/by-id/usb-Logitech_Extreme_3D_pro_00000000002A-joystick");

void init() {
    components[0] = new Drive();
    components[1] = new Manipulator(&js, servoDriver);
    client->start();
    server->start();
}

void stop() {
    
}

int main() {
    // initialize ROV parts
    init();
    ButtonPresses presses;
    
    while(true) {
        js.updatePresses();

        // update active component
        presses = js.getPresses();
        activeComponent->setActive(false);
        if(presses[2]) { // see JoystickMap.md for full list
            activeComponent = components[0];
        } else if(presses[3]) {
            activeComponent = components[1];
        }
        activeComponent->setActive(true);
        
        // Update each component
        for(Component* component : components) {
           component->Update();
        }
    }
    
    stop();
    return 0;
}
