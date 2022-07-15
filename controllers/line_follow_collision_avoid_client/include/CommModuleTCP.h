#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <iostream>
#include <sstream>
#include <thread>

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

/* include abstract module interface */
#include "ICommModule.h"

class CommModuleTCP : 
    public ICommModule
{

public:

    CommModuleTCP(int port = 1000);
    
    bool registerAtSupervisor(std::string robotName);
    void unregisterFromSupervisor(std::string reason = std::string("none"));

    bool requestPath(coordinate startXY, coordinate goalXY);
    bool receivePath(std::vector<coordinate>* path);

    bool reportCollision(coordinate startXY, coordinate goalXY, coordinate collisionXY);
    bool receiveAlternativePath(std::vector<coordinate>* path, int* msgTypeError = 0);

private:

    enum MessageType
    {
        REGISTER = 0,
        UNREGISTER,
        PATH,
        COLLISION, 
        INVALID = 99
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
    MessageType getMessageIdentifier(std::string msg);
    int getMessageTypeErrorCode(std::string msg);
    std::vector<std::string> msgStringSplit(std::string msg);

    SOCKET connectSocket;
    int wifiPort;
    static const size_t maxMsgLen = 512;
    std::string mClientName;
    bool socketApiInitialized = false;
};
