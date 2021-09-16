#include <iostream>
#include <components/Component.hpp>
#include <networking/TCPClientServer.hpp>

Component* components[2] = {nullptr, nullptr};
TCP_Client client;
TCP_Server server;

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