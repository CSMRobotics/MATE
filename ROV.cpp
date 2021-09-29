#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"

#include "TestManager.hpp"
#include "ServoTest.hpp"

Component* components[2] = {nullptr, nullptr};
TCP_Client* client = new TCP_Client();
TCP_Server* server = new TCP_Server();

void init() {
    components[0] = new Drive();
    client->start();
    server->start();
}

void doTests() {
    // add functionalities to test (NOTHING CAN BE BLOCKING, NO ASYNC FUNCTIONALITY IMPLEMENTED)
    TestManager::registerTest(new ServoTest());

    // actually test
    TestManager::runTests(5);
}

int main() {
    // UNCOMMENT FOR TESTING
    doTests();
    return 0;

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