#include "CommunicationModuleWifi.h"

CommunicationModuleWifi::CommunicationModuleWifi(int port)
{
    wifiPort = port;
    connectSocket = INVALID_SOCKET;
    mClientName = "undefined";
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

bool CommunicationModuleWifi::tryToConnectToSupervisor(std::string robotName)
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

        /*
        *   Try to send register message and receive ACK. 
        *   When sendRegistrationToSupervisor was successful, try to receive the acknowledge. 
        *   After maxRegisterAttempts of sending or receiving data the connection is set as failed
        */
        int maxRegisterAttempts = 5;

        for (int i = 0; i < maxRegisterAttempts; i++) {

            /* Try to send register message */
            if (sendRegistrationToSupervisor(robotName)) {

                /* Try to receive ACK */
                for (int i = 0; i < maxRegisterAttempts; i++) {

                    //std::this_thread::sleep_for(std::chrono::milliseconds(100));

                    /* Check for registration acknowledge */
                    if (receiveRegistrationAck())
                        return true;
                }

                return false;
            }
        }
                
    }

    return false;
}

bool CommunicationModuleWifi::sendMessage(const char* message, int msgLen)
{
    int sendResult;

    /* send message to client via tcp socket */
    sendResult = send(connectSocket, message, maxMsgLen, 0);

    if (sendResult == SOCKET_ERROR) {
        std::cerr << "CommunicationModuleWifi (" << mClientName << "): Failed to send message to server" << std::endl;
        return false;
    }

    return true;
}

char* CommunicationModuleWifi::receiveMessage(void)
{
    char recvBuffer[maxMsgLen];
    int recvSize = recv(connectSocket, recvBuffer, maxMsgLen, 0);

    if (recvSize == 0) {
        // std::cerr << "CommunicationModuleWifi (" << mClientName << "): No data received from server" << std::endl;
        return nullptr;
    }

    return recvBuffer;

}

bool CommunicationModuleWifi::sendRegistrationToSupervisor(std::string robotName)
{
    /* prepare register message */
    std::ostringstream stringStream;
    stringStream << MessageType::REGISTER << ";" << robotName;
    std::string msgString = stringStream.str();

    if (!sendMessage(msgString.c_str(), msgString.size())) {
        return false;
    }

    return true;
}

bool CommunicationModuleWifi::receiveRegistrationAck(void)
{
    char* msg = receiveMessage();

    if (msg != nullptr) {

        int msgIdentifier = msg[0] - '0';

        if (msgIdentifier == 0) {
            return true;
        }   
    }

    return false;
}

void CommunicationModuleWifi::unregisterFromSupervisor(std::string reason)
{
    /* prepare unregister message */
    std::ostringstream stringStream;
    stringStream << MessageType::UNREGISTER << ";" << reason;
    std::string msgString = stringStream.str();

    sendMessage(msgString.c_str(), msgString.size());
}

bool CommunicationModuleWifi::reportCollision(
    std::tuple<int, int> startXY,
    std::tuple<int, int> goalXY,
    std::tuple<int, int> collisionXY)
{
    /* prepare collision message */
    /* e.g. 2;1,2;6,0;3,2 */
    std::ostringstream stringStream;

    stringStream << MessageType::COLLISION << ";" 
        << std::get<0>(startXY)     << "," << std::get<1>(startXY)  << ";"
        << std::get<0>(goalXY)      << "," << std::get<1>(goalXY)   << ";"
        << std::get<0>(collisionXY) << "," << std::get<1>(collisionXY);

    std::string msgString = stringStream.str();

    return sendMessage(msgString.c_str(), msgString.size());
}