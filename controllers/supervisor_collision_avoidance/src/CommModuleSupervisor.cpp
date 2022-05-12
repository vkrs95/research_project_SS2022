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

void CommModuleTCPSocketServer::sendMessageToClient(std::string clientName, std::string message)
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

void CommModuleTCPSocketServer::socketListenerRoutine(void)
{
    std::cout << "Communication Module Supervisor: Waiting for connections on port " << mServerPort << "..." << std::endl;

    while (true) {

        /* periodically check for clients waiting to connect */
        mClientSocket = socketAccept(mListenSocket);

        if (mClientSocket > 0) {

            /* create new client thread, add it to thread pool and let it run */
            ClientCommHandler* clientThread = new ClientCommHandler(mClientSocket);
            mCommHandlerThreadPool.push_back(clientThread);
            
            //std::cout << "Connection ID " << int(mCommHandlerThreadPool.size() - 1) << " established on socket " << mClientSocket << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));	// TODO: how long should the thread wait until next check?
    }
}

void CommModuleTCPSocketServer::ClientCommHandler::socketCommunicationHandlerRoutine(void)
{
    /* start endless thread routine */
 
    while (true) {
        /* periodically check for message from client */
        receiveMessage();

        /* periodically check for ordered message to send to client */
        sendMessage();

        /* TODO: how long should the thread wait until next check ? */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));	
    }
}

CommModuleTCPSocketServer::ClientCommHandler::ClientCommHandler(int clientSocket)
{
    mClientSocket = clientSocket;
    mClientName = "undefined";

    commHandlingThread = new std::thread(&ClientCommHandler::socketCommunicationHandlerRoutine, this);
    commHandlingThread->detach();
}

void CommModuleTCPSocketServer::ClientCommHandler::receiveMessage(void)
{
    char recvBuffer[maxMsgLen];
    int recvSize = recv(mClientSocket, recvBuffer, maxMsgLen, 0);

    if (recvSize > 0) {

        std::string recvMsgStr(recvBuffer);

        /* might be first message sent by client containing client's name */
        if (mClientName.compare("undefined") == 0) {

            /* to get e.g. '1' -> 1 etc. */
            int msgIdentifier = recvBuffer[0] - '0';

            /* check if message is of type REGISTER */
            if (msgIdentifier == MessageType::REGISTER) {

                /* send registration ACK */
                registerNameAndSendACK(recvMsgStr);
                return;
            }
            /* if MessageType != REGISTER just continue adding message to inbox */
        }

        /* add received message to internal inbox */
        std::unique_lock<std::mutex> msgInboxLock(msgMutex);
        mMsgInbox.push_back(recvMsgStr);
        msgInboxLock.unlock();
    }
}

void CommModuleTCPSocketServer::ClientCommHandler::sendMessage(void)
{
    std::string lastMessage;
    int sendResult;

    while (!mMsgOutbox.empty()) {

        /* set local mutex and get oldest message from outbox */
        std::unique_lock<std::mutex> msgOutboxLock(msgMutex);
        lastMessage = mMsgOutbox.front();
        mMsgOutbox.pop_front();              // remove the message from list
        msgOutboxLock.unlock();

        /* send message to client via tcp socket */
        sendResult = send(mClientSocket, lastMessage.c_str(), lastMessage.size(), 0);

        if (sendResult == SOCKET_ERROR) {
            std::cerr << "ClientCommHandler: Failed to send message to " << mClientName << std::endl;
        }
    }
}

void CommModuleTCPSocketServer::ClientCommHandler::registerNameAndSendACK(std::string message)
{
    std::string extractNameStr = message;

    extractNameStr.erase(0, 2); // erase 2 character since first one is message type, second is separator

    mClientName = extractNameStr;
    std::cout << "ClientCommHandler: client '" << mClientName << "' registered on socket " << mClientSocket << "!" << std::endl;
    
    /* send ACK to client */
    std::string ackMsg = std::to_string(MessageType::REGISTER) +";ACK";
    addMsgToOutbox(ackMsg);
}

bool CommModuleTCPSocketServer::ClientCommHandler::addMsgToOutbox(std::string message)
{
    /* set local mutex and add passed message to outbox */
    std::unique_lock<std::mutex> msgOutboxLock(msgMutex);
    mMsgOutbox.push_back(message);
    msgOutboxLock.unlock();

    return true;
}

std::string CommModuleTCPSocketServer::ClientCommHandler::getInboxMessage(void)
{
    std::string lastMessage;

    if (mMsgInbox.empty()) {
        lastMessage = "";
    }
    else {
        /* at least one message in inbox, set local mutex and pop next message from inbox */
        std::unique_lock<std::mutex> msgInboxLock(msgMutex);
        
        lastMessage = mMsgInbox.front();    // get oldest message from FIFO
        mMsgInbox.pop_front();              // remove the message from list

        msgInboxLock.unlock();

    }
    return lastMessage;
}

std::string CommModuleTCPSocketServer::ClientCommHandler::getClientName(void)
{
    return mClientName;
}
