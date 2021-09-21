// implements UDP for all data transmission with fixed size packets
#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#define ADDRLEN = sizeof(address)

class TCP_Client {
public:
    TCP_Client(const char* address, int port);
    ~TCP_Client() = default;

    int getSock() const;
    int getPort() const;
    sockaddr_in getAddress() const;
private:
    int port;
    int sock;
    bool connected = false;
    char* buffer[1024];
    struct sockaddr_in address;
};

class TCP_Server {
public:
    TCP_Server() = default;
    ~TCP_Server();

    int getSock() const;
    int getPort() const;
    sockaddr_in getAddress() const;
private:
    int server_fd, new_socket, valread;
    struct sockaddr_in address;
    int opt = 1;
    char buffer[1024] = {0};
};