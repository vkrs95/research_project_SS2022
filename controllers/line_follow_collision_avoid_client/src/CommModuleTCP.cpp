#include "CommModuleTCP.h"

CommModuleTCP::CommModuleTCP(int port)
{
    wifiPort = port;
    connectSocket = INVALID_SOCKET;
    mClientName = "undefined";
}


bool CommModuleTCP::socketInit() {
#ifdef _WIN32 /* initialize the socket API */
    WSADATA info;
    if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
        fprintf(stderr, "Cannot initialize Winsock.\n");
        return false;
    }
#endif
    socketApiInitialized = true;
    return true;
}

bool CommModuleTCP::socketClose(int fd) {
#ifdef _WIN32
    return (closesocket(fd) == 0) ? true : false;
#else
    return (close(fd) == 0) ? true : false;
#endif
}

bool CommModuleTCP::socketCleanup() {
#ifdef _WIN32
    return (WSACleanup() == 0) ? true : false;
#else
    return true;
#endif
}

bool CommModuleTCP::socketSetNonBlocking(int fd) {
    if (fd < 0)
        return false;
#ifdef _WIN32
    unsigned long mode = 1; //  if iMode != 0, non-blocking mode is enabled
    return (ioctlsocket(fd, FIONBIO, &mode) == 0) ? true : false;
#else
    int flags = fcntl(fd, F_GETFL, 0) | O_NONBLOCK;
    return (fcntl(fd, F_SETFL, flags) == 0) ? true : false;
#endif
}

bool CommModuleTCP::socketWritable(void)
{
    /*
    *   When a socket is processing a connect call(nonblocking), a socket is writable if
    *   the connection establishment successfully completes.
    */
    fd_set wfds;
    struct timeval tv = { 1, 0 };   // timeval of 1 sec

    FD_ZERO(&wfds);
    FD_SET(connectSocket, &wfds);

    int number = select((int)connectSocket, NULL, &wfds, NULL, &tv);

    if (number == 0) {
        closesocket(connectSocket);
        connectSocket = INVALID_SOCKET;
        std::cerr << "Failed to get socket writable." << std::endl;
        return false;
    }

    return true;
}

bool CommModuleTCP::registerAtSupervisor(std::string robotName)
{
    if (!socketApiInitialized) {
        if (!socketInit()) {
            std::cerr << "Failed to init socket API." << std::endl;
            return false;
        }
    }

    if (!socketReady) {

        // set client name 
        mClientName = robotName;

        struct sockaddr_in address;

        memset(&address, 0, sizeof(struct sockaddr_in));
        address.sin_family = AF_INET;
        address.sin_port = htons((unsigned short)wifiPort);
        address.sin_addr.s_addr = inet_addr("127.0.0.1"); // INADDR_ANY;

        // Create a SOCKET for connecting to server
        connectSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        if (connectSocket == INVALID_SOCKET) {
            std::cerr << "socket failed with error: " << WSAGetLastError() << std::endl;
            WSACleanup();
            return false;
        }

        if (!socketSetNonBlocking((int)connectSocket)) {
            std::cerr << "Failed to set connect socket non-blocking." << std::endl;
            return false;
        }

        // Connect to server.
        int iResult = connect(connectSocket, (struct sockaddr*)&address, sizeof(struct sockaddr));

        /*
        *   Socket is non-blocking so connection is not completed immediately thus we
        *   expect a SOCKET_ERROR result in the first place and WSAEWOULDBLOCK as errno.
        *
        *   See: https://docs.microsoft.com/de-de/windows/win32/api/winsock2/nf-winsock2-wsaconnect?redirectedfrom=MSDN
        */
        if (iResult != SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK)
        {
            closesocket(connectSocket);
            connectSocket = INVALID_SOCKET;
            std::cerr << "The server is down... did not connect" << std::endl;
            return false;
        }

        /*  Continue with checking if socket is writable */
        if (!socketWritable())
            return false;

        socketReady = true;
    }

    /*
    *   Try to send register message and receive ACK. 
    *   When sendRegistrationToSupervisor was successful, try to receive the acknowledge. 
    *   After maxRegisterAttempts of sending or receiving data the connection is set as failed
    */
    int maxRegisterAttempts = 10;

    for (int i = 0; i < maxRegisterAttempts; i++) {

        /* Try to send register message */
        if (sendRegistrationToSupervisor(robotName)) {

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            /* Try to receive ACK */
            for (int i = 0; i < maxRegisterAttempts; i++) {

                /* Check for registration acknowledge */
                Message* msg = receiveMessage();

                if (msg != nullptr) {

                    if (msg->getType() == MessageType::REGISTER) {
                        if (msg->getErrorCode() == MessageErrorCode::SUCCESS)
                            return true;
                    }
                    else {
                        /* 
                        *   Received message is valid but not expected ACK message.
                        *   Abort procedure and send another registration message in next step
                        */
                        std::cout << "CommModuleTCP: error when trying to receive ACK msg. Received ID: " << static_cast<int>(msg->getType())
                            << ", Error Code: " << static_cast<int>(msg->getErrorCode()) << std::endl;

                        return false;
                    }
                }
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            /* error case: failed to receive ACK after maxRegisterAttempts attempts */
            break;  
        }
        else {

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::cout << "Error " << robotName.c_str() << ": maximum attempts of sending / receiving ACK exceeded." << std::endl;
    return false;
}

bool CommModuleTCP::sendMessage(Message* message)
{
    int sendResult;

    /* send message to client via tcp socket */
    sendResult = send(connectSocket, message->getMessageAsChar() , Message::MAX_MSG_LEN, 0);

    if (sendResult == SOCKET_ERROR) {
        std::cerr << "CommModuleTCP (" << mClientName << "): Failed to send message to server" << std::endl;
        return false;
    }

    return true;
}

Message* CommModuleTCP::receiveMessage(void)
{
    char recvBuffer[Message::MAX_MSG_LEN] = {};

    /* check socket if there is something to read */
    fd_set rfds;
    struct timeval tv = { 0, 100 };   // timeval of 100 ms

    FD_ZERO(&rfds);
    FD_SET(connectSocket, &rfds);

    /*
    *   Check the readability status of the communication socket. Readability means that queued 
    *   data is available for reading such that a call to recv is guaranteed not to block.
    */
    int number = select((int)connectSocket, &rfds, NULL, NULL, &tv);

    if (number != 0) {
        int recvSize = recv(connectSocket, recvBuffer, Message::MAX_MSG_LEN, 0);

        if (recvSize >= Message::MIN_MSG_LEN) {

            Message* msg = new Message(recvBuffer, recvSize);

            //std::cout << "CommModuleTCP(" << mClientName << ") : received message string'" << msg->getMessageAsChar() << "' with size " << recvSize << std::endl;
            return msg;
        }
    }

    // std::cerr << "CommModuleTCP (" << mClientName << "): No data received from server" << std::endl;
    return nullptr;
}

bool CommModuleTCP::sendRegistrationToSupervisor(std::string robotName)
{
    /* prepare register message */
    Message* msg = new Message(MessageType::REGISTER, robotName);

    if (!sendMessage(msg)) {
        return false;
    }

    return true;
}

bool CommModuleTCP::receiveRegistrationAck(void)
{
    Message* msg = receiveMessage();

    if (msg != nullptr) {

        if (msg->getType() == MessageType::REGISTER) {
            if(msg->getErrorCode() == MessageErrorCode::SUCCESS)
                return true;
        }   

        std::cout << "CommModuleTCP: error when trying to receive ACK msg. Received ID: " << static_cast<int>(msg->getType())
            << ", Error Code: " << static_cast<int>(msg->getErrorCode()) << std::endl;
    }

    return false;
}

void CommModuleTCP::unregisterFromSupervisor(std::string reason)
{
    /* prepare unregister message */
    Message* msg = new Message(MessageType::UNREGISTER, reason);

    sendMessage(msg);
}

bool CommModuleTCP::requestPath(coordinate startXY, coordinate goalXY)
{
    /* prepare path message */
    /* e.g. 2;1,2;6,0 */
    std::ostringstream stringStream;

    /* build path msg string */
    stringStream << std::get<0>(startXY) << "," << std::get<1>(startXY) << ";"
                << std::get<0>(goalXY) << "," << std::get<1>(goalXY);

    std::string msgString = stringStream.str();

    Message* msg = new Message(MessageType::PATH, msgString);

    return sendMessage(msg);
}

bool CommModuleTCP::receivePath(std::vector<coordinate>* path)
{
    Message* msg = receiveMessage();

    if (msg != nullptr) {

        if (msg->getType() == MessageType::PATH) {
            /* received supervisor response with path */
            *path = parsePath(msg->getPayload());

            return true;
        }
    }

    return false;
}

bool CommModuleTCP::reportCollision(
    coordinate startXY,
    coordinate goalXY,
    coordinate collisionXY)
{
    /* prepare collision message */
    /* e.g. 3;1,2;6,0;3,2 */
    std::ostringstream stringStream;

    /* build collision msg string */
    stringStream << std::get<0>(startXY)     << "," << std::get<1>(startXY)  << ";"
                << std::get<0>(goalXY)      << "," << std::get<1>(goalXY)   << ";"
                << std::get<0>(collisionXY) << "," << std::get<1>(collisionXY);

    std::string msgString = stringStream.str();
    
    Message* msg = new Message(MessageType::COLLISION, msgString);

    return sendMessage(msg);
}

bool CommModuleTCP::receiveAlternativePath(std::vector<std::tuple<int, int>>* path, int* msgState)
{
    Message* msg = receiveMessage();

    if (msg != nullptr) {
                
        /* get msg type as integer from msg */
        if (msg->getType() == MessageType::COLLISION) {
            /* received supervisor response with collision avoidance path */
            *path = parsePath(msg->getPayload());
            *msgState = static_cast<int>(msg->getErrorCode());

            return true;
        }
    }

    return false;
}

std::vector<coordinate> CommModuleTCP::parsePath(std::string msg)
{
    std::vector<coordinate> path;
    std::vector<std::string> subStrings = msgStringSplit(msg);

    for (int i = 0; i < subStrings.size(); i++) {
        /* convert coordinates from string and add them to path list */
        path.push_back(getCoordinateTuple(subStrings[i]));
    }

    return path;
}

coordinate CommModuleTCP::getCoordinateTuple(std::string tupleString)
{
    /* passed tuple string is expected to be build up: xCoord,yCoord */
    int xCoord = 0, yCoord = 0;
    size_t pos = tupleString.find(',');

    if (pos == std::string::npos) {
        // string does not contain comma delimiter
        return { 0,0 };
    }

    xCoord = std::stoi(tupleString.substr(0, pos));
    yCoord = std::stoi(tupleString.substr(pos + 1, tupleString.size() - pos + 1));

    return { xCoord, yCoord };
}

MessageType CommModuleTCP::getMessageIdentifier(std::string msg)
{
    std::vector<std::string> subStrings = msgStringSplit(msg);

    if (subStrings.size() < 1) {
        // msg does not contain semicolon delimiter
        return MessageType::INVALID;
    }

    /* get msg type as integer from msg */
    int msgIdentifier = std::stoi(subStrings.at(0));

    return static_cast<MessageType>(msgIdentifier);
}

std::vector<std::string> CommModuleTCP::msgStringSplit(std::string msg)
{
    std::string subStr;
    std::vector<coordinate> path;
    std::vector<std::string> subStrings;

    /* load message content into string stream */
    std::istringstream iss(msg);

    /* go through stream and extract substring between delimiter ';' */
    while (std::getline(iss, subStr, ';')) {
        subStrings.push_back(subStr);
    }

    return subStrings;
}

int CommModuleTCP::getMessageTypeErrorCode(std::string msg)
{
    std::vector<std::string> subStrings = msgStringSplit(msg);

    if (subStrings.size() < 2) {
        // msg does not contain semicolon delimiter
        return 99; // ERROR CODE ?
    }

    std::cout << "Error Code as string: " << subStrings.at(1) << " from " << msg << std::endl;

    /* get msg type as integer from msg */
    return std::stoi(subStrings.at(1));
}