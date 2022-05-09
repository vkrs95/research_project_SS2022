#include "CommunicationModuleWifi.h"

CommunicationModuleWifi::CommunicationModuleWifi(int port)
{
    wifiPort = port;
    connectSocket = INVALID_SOCKET;
}


bool CommunicationModuleWifi::socketInit() {
#ifdef _WIN32 /* initialize the socket API */
    WSADATA info;
    if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
        fprintf(stderr, "Cannot initialize Winsock.\n");
        return false;
    }
#endif
    return true;
}

bool CommunicationModuleWifi::socketClose(int fd) {
#ifdef _WIN32
    return (closesocket(fd) == 0) ? true : false;
#else
    return (close(fd) == 0) ? true : false;
#endif
}

bool CommunicationModuleWifi::socketCleanup() {
#ifdef _WIN32
    return (WSACleanup() == 0) ? true : false;
#else
    return true;
#endif
}

bool CommunicationModuleWifi::tryToConnectToSupervisor()
{
    if (socketInit()) {

        struct sockaddr_in address;

        memset(&address, 0, sizeof(struct sockaddr_in));
        address.sin_family = AF_INET;
        address.sin_port = htons((unsigned short) wifiPort);
        address.sin_addr.s_addr = inet_addr("127.0.0.1"); // INADDR_ANY;

        // Create a SOCKET for connecting to server
        connectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        if (connectSocket == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return false;
        }
        
        // Connect to server.
        int iResult = connect(connectSocket, (struct sockaddr*)&address, sizeof(struct sockaddr));

        if (iResult == SOCKET_ERROR)
        {
            closesocket(connectSocket);
            connectSocket = INVALID_SOCKET;
            printf("The server is down... did not connect");
            return false;
        }

        return true;
    }

    return false;
}