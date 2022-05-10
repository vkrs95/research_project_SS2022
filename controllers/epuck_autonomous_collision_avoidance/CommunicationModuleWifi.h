#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>
#include <sstream>
#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h> /* definition of inet_ntoa */
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

class CommunicationModuleWifi
{

public:

    CommunicationModuleWifi(int port = 1000);
    bool tryToConnectToSupervisor(std::string robotName);
    bool registerToSupervisor(std::string robotName);
    void unregisterFromSupervisor(std::string reason = std::string("none"));
    bool reportCollision(std::tuple<int, int> startXY, 
                            std::tuple<int, int> goalXY, 
                            std::tuple<int, int> collisionXY);

private:

    enum MessageType
    {
        REGISTER = 0,
        UNREGISTER,
        COLLISION
    };

    bool socketInit();
    bool socketClose(int fd);
    bool socketCleanup();
    bool sendMessage(const char* message, int msgLen);

    SOCKET connectSocket;
    int wifiPort;
    int maxMsgLen = 512;
    std::string mClientName;
};
