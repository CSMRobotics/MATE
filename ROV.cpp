#include <iostream>
#include "Drive.hpp"
#include "Manipulator.hpp"
#include "TCPClientServer.hpp"
#include "Joystick.hpp"
#include "ServoDriver.hpp"

#include "CSMUtil.hpp"

Component* components[1];
// TCP_Client* client = new TCP_Client();
//TCP_Server* server = new TCP_Server();
// ServoDriver* servoDriver = new ServoDriver();
// Joystick js = Joystick("/dev/input/by-id/usb-Logitech_Extreme_3D_pro_00000000002A-joystick");

void init() {
    // components[0] = new Drive();
    // components[0] = new Manipulator(&js, servoDriver);
    // client->start();
    // server->start();
}

void test() {
    csmutil::Quaternionf q(1,2,3,4);
    csmutil::Quaternionf p(4,3,2,1);

    std::cout << q << std::endl;
    std::cout << p << std::endl;
    csmutil::Quaternionf result(q*p);

    std::cout << result << std::endl;
    std::cout << result.getNorm() << std::endl;
    std::cout << result.getAsUnit() << std::endl;
    std::cout << result.getConjugate() << std::endl;
    std::cout << result.getReciprocal() << std::endl;
    std::pair<csmutil::Vector3f, float> axisangle = result.getAxisAngle();
    std::cout << (axisangle.first) << ' ' << axisangle.second << std::endl;

    exit(0);
}

int main() {
    // initialize ROV parts
    test();
    init();
    
    // start update loop
    bool quit = false;
    // while(!quit) {
    //     js.updatePresses();
    //     for(Component* component : components) {
    //         component->Update();
    //     }
    // }
    
    return 0;
}
