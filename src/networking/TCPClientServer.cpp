#include <include/networking/TCPClientServer.hpp>

TCP_Client::TCP_Client(const char address[], int port) {
    this->sock = socket(AF_INET, SOCK_STREAM, 0);
    if(this->sock < 0) {
        printf("\nSocket creation error\n"); // TODO: replace with logger
    }

    this->address.sin_family = AF_INET;
    this->address.sin_port = htons(port);
    if(inet_pton(AF_INET, address, &this->address.sin_addr) <=0) {
        printf("\nInvalid Address\n"); // TODO: replace with logger
    }

    float retry_s = 0;
    while(!this->connected) {
        if(connect(sock, (struct sockaddr *)&this->address, sizeof(this->address)) < 0 ) {
            retry_s = exponentialBackoff(retry_s);
            printf("\nConnection Failed...Retrying in %f.2f seconds", retry_s);
            sleep(retry_s);
            continue;
        }
        else
            this->connected = true;
    }
}

float exponentialBackoff(float retry_s) {
    if(retry_s == 0)
        return .250;
    return retry_s * 2;
}