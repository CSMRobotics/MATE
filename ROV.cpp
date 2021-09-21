#include <iostream>
#include "Component.hpp"
#include "TCPClientServer.hpp"

Component* components[2] = {nullptr, nullptr};
TCP_Client* client = new TCP_Client("10.0.0.2", 7777);
TCP_Server* server;

void init() {
    components[0];
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