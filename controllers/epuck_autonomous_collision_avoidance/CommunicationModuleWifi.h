#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

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

using coordinate = std::tuple<int, int>;

class CommunicationModuleWifi
{

public:

    /*
    *   AbstractCommunicationModule
    *   registerAtSupervisor
    *   unregisterFromSupervisor
    *   reportCollision
    *   receiveCollisionReply
    */


    CommunicationModuleWifi(int port = 1000);
    bool tryToConnectToSupervisor(std::string robotName);
    void unregisterFromSupervisor(std::string reason = std::string("none"));
    bool reportCollision(std::tuple<int, int> startXY, 
                            std::tuple<int, int> goalXY, 
                            std::tuple<int, int> collisionXY);
    bool receiveCollisionReply(std::vector<std::tuple<int, int>>* path);

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
    bool socketSetNonBlocking(int fd);
    bool sendMessage(const char* message, size_t msgLen);
    bool sendRegistrationToSupervisor(std::string robotName);
    bool receiveRegistrationAck(void);
    std::string receiveMessage(void);
    std::vector<coordinate> parsePath(std::string msg);
    coordinate getCoordinateTuple(std::string tupleString);

    SOCKET connectSocket;
    int wifiPort;
    static const size_t maxMsgLen = 512;
    std::string mClientName;
    bool socketApiInitialized = false;
};
