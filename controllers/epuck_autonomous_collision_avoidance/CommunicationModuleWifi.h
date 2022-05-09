#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
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
    bool socketInit();
    bool socketClose(int fd);
    bool socketCleanup();
    bool tryToConnectToSupervisor();

private:

    SOCKET connectSocket;
    int wifiPort;
};
