#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"

#include "ServoTest.hpp"

Component* components[2] = {nullptr, nullptr};
TCP_Client* client = new TCP_Client();
TCP_Server* server = new TCP_Server();

void init() {
    components[0] = new Drive();
    client->start();
    server->start();
}

int main() {
    TEST(); // UNCOMMENT FOR TESTING

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