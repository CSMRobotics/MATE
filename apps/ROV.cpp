#include <iostream>
#include <include/components/Component.hpp>
#include <include/networking/UDPClientServer.hpp>

Component* components[2] = {nullptr, nullptr};
udp_client client;
udp_server server;

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