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

#define NETWORK_PORT 7777
#define DRIVERSTATION_ADDRESS "127.0.0.1"

class TCP_Client {
public:
    TCP_Client();
    ~TCP_Client() = default;

    void start();
    bool isStarted();
    void stop();

    void sendMessage();
private:
    int sock;
    char buffer[4096];
    struct sockaddr_in address;
    bool started;
    bool shouldThreadBeRunning = true;
    std::thread client;
    std::priority_queue<std::string> sendQueue;

    void handleConnection(bool& running);
};

class TCP_Server {
public:
    TCP_Server();
    ~TCP_Server() = default;

    void start();
    bool isStarted();
    void stop();
private:
    int listeningSocket, clientSocket;
    char host[NI_MAXHOST], service[NI_MAXSERV];
    sockaddr_in address, client;
    socklen_t clientSize = sizeof(client);
    char buffer[4096] = {0};
    bool started;
    bool shouldThreadBeRunning = true;
    std::thread server;

    void handleConnection(bool& running);
};

#endif // TCPCLIENTSERVER_HPP