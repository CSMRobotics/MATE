#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"

#include "TestManager.hpp"
#include "ServoTest.hpp"

Component* components[2];
TCP_Client* client = new TCP_Client();
TCP_Server* server = new TCP_Server();

void init() {
    // components[0] = new Drive();
    client->start();
    server->start();
}

int main() {
    // initialize ROV parts
    init();

    // start update loop
    bool quit = false;
    while(!quit) {
        for(Component* component : components) {
            component->Update();
        }
    }

    return 0;
}