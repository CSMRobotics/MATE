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

#define PORT_RECV 7777
#define PORT_SEND 7778
#define DRIVERSTATION_ADDRESS "10.0.0.1"

class TCP_Client {
public:
    TCP_Client();
    ~TCP_Client() = default;

    void start();
    void stop();
private:
    int sock;
    char buffer[4096];
    struct sockaddr_in address;
};

class TCP_Server {
public:
    TCP_Server();
    ~TCP_Server() = default;

    void start();
    void stop();
private:
    int listeningSocket, clientSocket;
    char host[NI_MAXHOST], service[NI_MAXSERV];
    sockaddr_in address, client;
    socklen_t clientSize = sizeof(client);
    char buffer[4096] = {0};

    void handleConnection();
};

#endif // TCPCLIENTSERVER_HPP