#include "CommModuleSupervisor.h"


CommModuleTCPSocketServer::CommModuleTCPSocketServer(int port)
{
    mServerPort = port;

    /* default initialisation of non-blocking socket server */
    mListenSocket = createSocketServer(mServerPort);
    
    if (!socketSetNonBlocking(mListenSocket)) {
        std::cerr << "Communication Module Supervisor: Failed to set server socket non blocking." << std::endl;
        return;
    }

    /* create listener thread to accept incoming connection requests */
    listenerThread = new std::thread(&CommModuleTCPSocketServer::socketListenerRoutine, this);
    listenerThread->detach();
}

CommModuleTCPSocketServer::~CommModuleTCPSocketServer()
{
    if (!socketClose(mListenSocket)) {
        std::cerr << "Communication Module Supervisor: Failed to properly close server socket ." << std::endl;
        return;
    }

    std::cout << "Communication Module Supervisor: deinitializing..." << std::endl;
}

bool CommModuleTCPSocketServer::socketInit() {
#ifdef _WIN32 /* initialize the socket API */
    WSADATA info;
    if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
        std::cerr << "Communication Module Supervisor: Cannot initialize Winsock." << std::endl;
        return false;
    }
#endif
    return true;
}

bool CommModuleTCPSocketServer::socketSetNonBlocking(int fd) {
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

int CommModuleTCPSocketServer::socketAccept(int server_fd) {
    int cfd;
    //struct sockaddr_in client;
    //struct hostent* client_info;

#ifndef _WIN32
    socklen_t asize;
#else
    int asize;
#endif
    asize = sizeof(struct sockaddr_in);
    cfd = (int)accept(server_fd, NULL, NULL); // accept(server_fd, (struct sockaddr*)&client, &asize);

    if (cfd == -1) {
#ifdef _WIN32
        int e = WSAGetLastError();
        if (e == WSAEWOULDBLOCK)
            return 0;
        std::cerr << "Communication Module Supervisor: Accept error: " << e << std::endl;
#else
        if (errno == EWOULDBLOCK)
            return 0;
        std::cerr << "Communication Module Supervisor: Accept error: " << errno << std::endl;
#endif
        return -1;
    }

    //client_info = gethostbyname((char*)inet_ntoa(client.sin_addr));
    //printf("Accepted connection from: %s.\n", client_info->h_name);

    return cfd;
}

bool CommModuleTCPSocketServer::socketClose(int fd) {
#ifdef _WIN32
    return (closesocket(fd) == 0) ? true : false;
#else
    return (close(fd) == 0) ? true : false;
#endif
}

bool CommModuleTCPSocketServer::socketCleanup() {
#ifdef _WIN32
    return (WSACleanup() == 0) ? true : false;
#else
    return true;
#endif
}

int CommModuleTCPSocketServer::createSocketServer(int port) {
    int sfd, rc;
    struct sockaddr_in address;

    if (!socketInit())
        return -1;

    sfd = (int)socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // IPPROTO_TCP instead of 0 ?

    if (sfd == -1) {
        std::cerr << "Communication Module Supervisor: Cannot create socket." << std::endl;
        return -1;
    }

    memset(&address, 0, sizeof(struct sockaddr_in));
    address.sin_family = AF_INET;
    address.sin_port = htons((unsigned short)port);
    address.sin_addr.s_addr = inet_addr("127.0.0.1"); // INADDR_ANY;

    rc = bind(sfd, (struct sockaddr*)&address, sizeof(struct sockaddr));

    if (rc == -1) {
        std::cerr << "Communication Module Supervisor: Cannot bind port " << port << std::endl;
        socketClose(sfd);
        return -1;
    }
    if (listen(sfd, 1) == -1) {
        std::cerr << "Communication Module Supervisor: Cannot listen for connections" << std::endl;
        socketClose(sfd);
        return -1;
    }

    return sfd;
}

void CommModuleTCPSocketServer::sendMessageToClient(std::string clientName, Message* message)
{
    std::string currentClient;

    for (ClientCommHandler* clientThread : mCommHandlerThreadPool) {
        currentClient = clientThread->getClientName();

        if (currentClient == clientName) {
            /* we found the correct client thread, forward message to its outbox */
            clientThread->addMsgToOutbox(message);
            break;
        }
    }
}

bool CommModuleTCPSocketServer::checkClientNotifications(
    std::list<PathNotification>* pathNotifications, 
    std::list<CollisionNotification>* collisionNotifications)
{
    Message* currentMsg;
    std::vector<ClientCommHandler*>::iterator it;

    it = mCommHandlerThreadPool.begin();

    /* iterate through comm handlers of all registered clients */
    while (it != mCommHandlerThreadPool.end()) {
        
        ClientCommHandler* clientThread = *it;

        currentMsg = clientThread->getInboxMessage();

        if (currentMsg != nullptr)
        {
            switch (currentMsg->getType())
            {
            case MessageType::PATH:
                pathNotifications->push_back(
                    PathNotification(clientThread->getClientName(), currentMsg->getPayload()));
                break;
            case MessageType::COLLISION:
                collisionNotifications->push_back(
                    CollisionNotification(clientThread->getClientName(), currentMsg->getPayload()));
                break;
            default:
                break;
            }
        } 
        else {
            it++;   // move to next client handler
        }
    }

    /* return true if at least one notfication has been added, else false */
    return !(collisionNotifications->empty() && pathNotifications->empty());

}

void CommModuleTCPSocketServer::sendCollisionMessageReply(std::map<std::string, std::vector<coordinate>> clientPaths)
{
    Message* msg;

    /* prepare message for each client */
    for (auto clientPath : clientPaths) {
        std::string pathMsgString = buildPathMsgString(clientPath.second, MessageType::COLLISION);
        msg = new Message(pathMsgString);

        sendMessageToClient(clientPath.first, msg);
    }
}

void CommModuleTCPSocketServer::sendPathMessage(std::string clientName, std::vector<coordinate> clientPath)
{
    Message* msg;

    /* build message string for client */
    std::string pathMsgString = buildPathMsgString(clientPath, MessageType::PATH);
    msg = new Message(pathMsgString);

   sendMessageToClient(clientName, msg);
   
}

std::tuple<int, int> CommModuleTCPSocketServer::getCoordinatesTuple(std::string tupleString)
{
    /* passed tuple string is expected to be build up: xCoord,yCoord */
    int xCoord = 0, yCoord = 0;
    size_t pos = tupleString.find(',');

    if (pos == std::string::npos) {
        // string does not contain comma delimiter
        return { 0,0 };
    }

    xCoord = std::stoi(tupleString.substr(0, pos ));
    yCoord = std::stoi(tupleString.substr(pos + 1, tupleString.size() - pos + 1));

    return { xCoord, yCoord };
}

void CommModuleTCPSocketServer::socketListenerRoutine(void)
{
    std::cout << "Communication Module Supervisor: Waiting for connections on port " << mServerPort << "..." << std::endl;
    unsigned int lifetime = 0;
    unsigned int sleeptime = 100; // ms
    unsigned int timeout = sleeptime * 50; // timeout after 5s runtime

    while (true) {

        /* periodically check for clients waiting to connect */
        mClientSocket = socketAccept(mListenSocket);

        if (mClientSocket > 0) {

            /* create new client thread, add it to thread pool and let it run */
            ClientCommHandler* clientThread = new ClientCommHandler(mClientSocket);
            mCommHandlerThreadPool.push_back(clientThread);
            
            //std::cout << "Connection ID " << int(mCommHandlerThreadPool.size() - 1) << " established on socket " << mClientSocket << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));	// TODO: how long should the thread wait until next check?

        if ((lifetime += sleeptime) >= timeout)
            break;
    }

    std::cout << "Communication Module Supervisor: Socket listener thread shutting down. No more connections accepted on port " << mServerPort << "." << std::endl;
}

std::string CommModuleTCPSocketServer::buildPathMsgString(std::vector<coordinate> path, MessageType msgType)
{
    std::stringstream sStrm;

    /* build string containing type and path, separated with semicolon */
    sStrm << (int)msgType << ";";

    /* each coordinate is parsed in pattern: 'x,y;x,y;x,y'*/
    for (coordinate c : path) {
        sStrm << std::get<0>(c) << "," << std::get<1>(c) << ";";
    }
    sStrm << "\0";
    return sStrm.str();
}


/***********************************************************
*
*	Implementation of nested class PathNotification
*
***********************************************************/

CommModuleTCPSocketServer::PathNotification::PathNotification(std::string clientName, std::string pathMsg)
{
    /* client got a message in its inbox */
    std::string subStr;
    std::vector<std::string> subStrings;

    /* load message content into string stream */
    std::istringstream iss(pathMsg);

    /* go through stream and extract substring between delimiter */
    while (std::getline(iss, subStr, ';')) {
        subStrings.push_back(subStr);
    }

    /* we need two substrings for start and goal node */
    if (subStrings.size() != 2) {
        std::cerr << "CommModuleTCPSocketServer: handler message contains invalid number of nodes. (" << subStrings.size() << ")." << std::endl;
        // go on with next message
    }
    else {
        /* assign values to struct */
        this->clientName = clientName;
        this->startNode = getCoordinatesTuple(subStrings.at(0));
        this->goalNode = getCoordinatesTuple(subStrings.at(1));
    }
}


/***********************************************************
*
*	Implementation of nested class CollisionNotification
*
***********************************************************/

CommModuleTCPSocketServer::CollisionNotification::CollisionNotification(std::string clientName, std::string collisionMsg)
{
    /* client got a message in its inbox */
    std::string subStr;
    std::vector<std::string> subStrings;

    /* load message content into string stream */
    std::istringstream iss(collisionMsg);

    /* go through stream and extract substring between delimiter */
    while (std::getline(iss, subStr, ';')) {
        subStrings.push_back(subStr);
    }

    /* we need three substrings for start, goal and collision node */
    if (subStrings.size() != 3) {
        std::cerr << "CommModuleTCPSocketServer: handler message contains invalid number of nodes. (" << subStrings.size() << ")." << std::endl;
        // go on with next message
    }
    else {
        /* assign values to struct */
        this->clientName = clientName;
        this->startNode = getCoordinatesTuple(subStrings.at(0));
        this->goalNode = getCoordinatesTuple(subStrings.at(1));
        this->collisionNode = getCoordinatesTuple(subStrings.at(2));
    }
}
