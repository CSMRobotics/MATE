#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"
#include "Joystick.hpp"
#include "ServoDriver.hpp"
#include "IMU.hpp"

Component* components[2];
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
    
    while(true) {
        js.updatePresses();
        for(Component* component : components) {
           component->Update();
        }
    }
    
    stop();
    return 0;
}
