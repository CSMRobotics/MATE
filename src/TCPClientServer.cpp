#include "TCPClientServer.hpp"

TCP_Server::TCP_Server() {
    // create a socket
    listeningSocket = socket(AF_INET, SOCK_STREAM, 0);
    if(listeningSocket == -1) {
        std::cout << "Unable to aquire socket. Quitting and dumping any locks.";
        std::abort();
    }

    // fill sockaddr_in
    address.sin_family = AF_INET;
    address.sin_port = htons(NETWORK_PORT);
    inet_pton(AF_INET, "0.0.0.0", &address.sin_addr);

    // bind socket to port
    bind(listeningSocket, (sockaddr*)&address, sizeof(address));
}

void TCP_Server::start() {
    started = true;
    // tell winsock the socket is ready for listening
    listen(listeningSocket, SOMAXCONN);

    // wait for connection
    std::cout << "Listening for connections on 0.0.0.0:" << NETWORK_PORT << std::endl;
    clientSocket = accept(listeningSocket, (sockaddr*)&client, &clientSize);

    memset(host, 0, NI_MAXHOST);
    memset(service, 0, NI_MAXSERV);

    // report that a connection has been established
    if(getnameinfo((sockaddr*)&client, sizeof(client), host, NI_MAXHOST, service, NI_MAXSERV, 0) == 0) {
        std::cout << host << " connected on port " << service << std::endl;
    }
    else {
        inet_ntop(AF_INET, &client.sin_addr, host, NI_MAXHOST);
        std::cout << host << " connected on port " << ntohs(client.sin_port) << std::endl;
    }

    // close the listening socket
    close(listeningSocket);

    // start handling thread
    server = std::thread(&TCP_Server::handleConnection, this, std::ref(shouldThreadBeRunning));
}

bool TCP_Server::isStarted() {
    return started;
}

void TCP_Server::handleConnection(bool& running) {
    // wait for messages
    while(running) {
        std::cout << "Waiting for message..." << "\n";
        memset(buffer, 0, 4096);
        
        // wait for client to send
        // TODO: make nonblocking
        int bytesReceived = recv(clientSocket, buffer, 4096, 0);
        switch(bytesReceived) {
            case -1:
                std::cerr << "Error in recv(). Quitting" << std::endl;
                return;
            case 0:
                std::cout << "Client disconnected" << std::endl;
                return;
            default:
                std::cout << "CLIENT> " << std::string(buffer, 0, bytesReceived) << "\n";
                break;
        }

        // TESTING ONLY
        // echo back to client
        send(clientSocket, buffer, bytesReceived + 1, 0);
    }

    // clear buffer
    memset(buffer, 0, 4096);
    // set buffer to indicate server is exiting;
    std::string toCpy = "Server Exiting";
    memcpy(buffer, toCpy.c_str(), 14);
    // send exit message to client
    send(clientSocket, buffer, 14, 0);

    close(clientSocket);
}

void TCP_Server::stop() {
    shouldThreadBeRunning = false;
    server.join();
}

TCP_Client::TCP_Client() {
    started = true;
    // create socket
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock == -1) {
        // THROW ERROR
    }

    // create address structure for server we are connecting to
    address.sin_family = AF_INET;
    address.sin_port = htons(NETWORK_PORT);
    inet_pton(AF_INET, DRIVERSTATION_ADDRESS, &address.sin_addr);
}

void TCP_Client::start() {
    // connect to server
    std::cout << "Connecting to " << DRIVERSTATION_ADDRESS << ":" << NETWORK_PORT << std::endl;
    int connectionResult = -1;
    do {
        connectionResult = connect(sock, (sockaddr*)&address, sizeof(address));
    } while(connectionResult == -1);

    client = std::thread(&TCP_Client::handleConnection, this, std::ref(shouldThreadBeRunning));

    // close socket
    close(sock);
}

void TCP_Client::handleConnection(bool& running) {
    // TESTING ONLY BELOW THIS LINE
    // TODO: REPLACE WITH PROPER SENDING TO DRIVERSTATION CLIENT
    std::string userInput;
    while(shouldThreadBeRunning) {
        // get input from user
        std::cout << "> ";
        getline(std::cin, userInput);

        // send to server
        int sendResult = send(sock, userInput.c_str(), userInput.size() + 1, 0);
        if(sendResult == -1) {
            std::cout << "Error occured while sending. Unable to deliver\r\n";
            continue;
        }

        // wait for echo
        memset(buffer, 0, 4096);
        int bytesReceived = recv(sock, buffer, 4096, 0);
        if(bytesReceived == -1) {
            std::cout << "Error getting echo from server\r\n";
        }
        else {
            std::cout << "SERVER> " << std::string(buffer, bytesReceived) << "\r\n";
        }
    }
}

bool TCP_Client::isStarted() {
    return started;
}

void TCP_Client::stop() {
    shouldThreadBeRunning = false;
    client.join();
    close(sock);
}

