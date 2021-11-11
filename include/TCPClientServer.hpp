#ifndef TCPCLIENTSERVER_HPP
#define TCPCLIENTSERVER_HPP

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <thread>
#include <queue>
#include <bitset>

#include "Joystick.hpp"

class Joystick;

#define NETWORK_PORT 7777
#define DRIVERSTATION_ADDRESS "10.0.0.1"

#define MSB 0x80000000
#define HEADER 0xFF000000
#define JOYSTICK_INDEX 0x1F000000
#define METADATA 0x00FF0000
#define DATA 0x0000FFFF

class TCP_Client {
public:
    TCP_Client();
    ~TCP_Client() = default;

    void start();
    void stop();

    void sendMessage();
private:
    int sock;
    char buffer[32];
    struct sockaddr_in address;
    bool shouldThreadBeRunning = true;
    std::thread client;
    std::priority_queue<uint32_t> sendQueue;

    void handleConnection(std::reference_wrapper<bool> running);
};

class TCP_Server {
public:
    TCP_Server();
    ~TCP_Server() = default;

    void registerJoystick(Joystick* joystick);

    void start();
    void stop();
private:
    int listeningSocket, clientSocket;
    char host[NI_MAXHOST], service[NI_MAXSERV];
    sockaddr_in address, client;
    socklen_t clientSize = sizeof(client);
    char buffer[32] = {0};
    bool shouldThreadBeRunning = true;
    std::thread server;

    Joystick* joystick;

    void handleConnection(std::reference_wrapper<bool> running);
    void decodeMessage(char* buffer);
};

#endif // TCPCLIENTSERVER_HPP
