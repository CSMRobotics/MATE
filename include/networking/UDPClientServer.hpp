// implements UDP for all data transmission with fixed size packets
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdexcept>

class udp_client {
public:
    udp_client(const std::string &addr, int port);
    ~udp_client();

    int getSocket() const;
    int getPort() const;
    std::string get_addr() const;

    int send(const char* msg, size_t size);
private:
    int f_socket;
    int f_port;
    std::string f_addr;
    struct addrinfo* f_addrinfo;
};

class udp_server {
public:
    udp_server(const std::string &addr, int port);
    ~udp_server();

    int getSocket() const;
    int getPort() const;
    std::string get_addr() const;

    int recv(char* msg, size_t max_size);
    int wait_recv(char* msg, size_t max_size, int max_wait_ms);

private:
    int f_socket;
    int f_port;
    std::string f_addr;
    struct addrinfo* f_addrinfo;
};