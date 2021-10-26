#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"
#include "Joystick.hpp"

Component* components[2];
TCP_Client* client = new TCP_Client();
TCP_Server* server = new TCP_Server();
controller::Joystick js = controller::Joystick("/dev/input/js0");

void init() {
    // components[0] = new Drive();
    client->start();
    server->start();
}

void testJoystick() {
    sleep(1);
    while(js.isFound()) {
        js.update();
        
        controller::ButtonPresses presses = js.getPresses();

        std::cout << presses << '\n';
        std::cout << "sleeping for 5 seconds" << '\n';
        sleep(5);
    }
}

int main() {
    testJoystick();
    // initialize ROV parts
    init();

    // start update loop
    bool quit = false;
    while(!quit) {
        js.update(); // update controller button presses
        for(Component* component : components) {
            component->Update();
        }
    }
    
    return 0;
}