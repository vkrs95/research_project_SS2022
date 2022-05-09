#include "supvervisor_collision_avoidance.h"

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    robot = new Robot();
    emitter = robot->getEmitter("emitter");

    /*************************************/
    mTimeStep =(unsigned int)robot->getBasicTimeStep();

    const int DEFAULT_BUFLEN = 512;
    char sendbuf[DEFAULT_BUFLEN];
    int sendbuflen = DEFAULT_BUFLEN;

    /*
    *   how to:
    *   communication between one supvervisor entity and multiple robots.
    *   communication medium is wifi. Supervisor opens up a server on a specific port.
    *   Robots try to connect to the server.
    *   When setting up a socket server, the result is a SFD value. 
    *   The clients need this SFD value in order to connect to the correct port.
    *   -> how to communicate this SFD value from supervisor entity to all clients ?
    *   -> using emitter/receiver mechanism by webots as seen in webots' soccer example?
    */
    constexpr auto port = 1000;
    int mSocket;

    /* open a server on port 1000 */
    mSocket = create_socket_server(port);

    /* try to set socket non blocking */
    if (!socket_set_non_blocking(mSocket)) {
        /* failed to set server non-blocking */
        return -1;
    }

    int clientSocket = 0;
    
    printf("Supervisor: Waiting for connections on port %d...\n", port);

    sendbuf[0] = '4';
    sendbuf[1] = '2';
    sendbuf[2] = '\0';

    while (robot->step(mTimeStep) != -1) {
        //socket_send(clientSocket, sendbuf, sendbuflen);
        //printf("Sent data %d...\n", socket_send(clientSocket, sendbuf, sendbuflen));

        //emitter->send(&mSocket, sizeof(mSocket));

        //robotActiveWait(1000);

        clientSocket = socket_accept(mSocket);

        if (clientSocket > 0) {
            clientList.push_back(clientSocket);
            printf("connection ID %d established on socket %d !\n", int(clientList.size() - 1), clientSocket);
        }

    }

    /*  optional cleanup  */
    delete robot;

    return 0;
}


bool socket_init() {
#ifdef _WIN32 /* initialize the socket API */
    WSADATA info;
    if (WSAStartup(MAKEWORD(1, 1), &info) != 0) {
        fprintf(stderr, "Cannot initialize Winsock.\n");
        return false;
    }
#endif
    return true;
}

bool socket_set_non_blocking(int fd) {
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

int socket_accept(int server_fd) {
    int cfd;
    //struct sockaddr_in client;
    //struct hostent* client_info;

#ifndef _WIN32
    socklen_t asize;
#else
    int asize;
#endif
    asize = sizeof(struct sockaddr_in);
    cfd = (int) accept(server_fd, NULL, NULL); // accept(server_fd, (struct sockaddr*)&client, &asize);

    if (cfd == -1) {
#ifdef _WIN32
        int e = WSAGetLastError();
        if (e == WSAEWOULDBLOCK)
            return 0;
        fprintf(stderr, "Accept error: %d.\n", e);
#else
        if (errno == EWOULDBLOCK)
            return 0;
        fprintf(stderr, "Accept error: %d.\n", errno);
#endif
        return -1;
    }

    //client_info = gethostbyname((char*)inet_ntoa(client.sin_addr));
    //printf("Accepted connection from: %s.\n", client_info->h_name);

    return cfd;
}

bool socket_close(int fd) {
#ifdef _WIN32
    return (closesocket(fd) == 0) ? true : false;
#else
    return (close(fd) == 0) ? true : false;
#endif
}

bool socket_cleanup() {
#ifdef _WIN32
    return (WSACleanup() == 0) ? true : false;
#else
    return true;
#endif
}

int create_socket_server(int port) {
    int sfd, rc;
    struct sockaddr_in address;

    if (!socket_init())
        return -1;

    sfd = (int) socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // IPPROTO_TCP instead of 0 ?

    if (sfd == -1) {
        fprintf(stderr, "Cannot create socket.\n");
        return -1;
    }

    memset(&address, 0, sizeof(struct sockaddr_in));
    address.sin_family = AF_INET;
    address.sin_port = htons((unsigned short)port);
    address.sin_addr.s_addr = inet_addr("127.0.0.1"); // INADDR_ANY;

    rc = bind(sfd, (struct sockaddr*)&address, sizeof(struct sockaddr));

    if (rc == -1) {
        fprintf(stderr, "Cannot bind port %d.\n", port);
        socket_close(sfd);
        return -1;
    }
    if (listen(sfd, 1) == -1) {
        fprintf(stderr, "Cannot listen for connections.\n");
        socket_close(sfd);
        return -1;
    }

    return sfd;
}

int socket_send(SOCKET socket, char* sendBuffer, int sendBufferLen)
{
    int sendResult;

    sendResult = send(socket, sendBuffer, sendBufferLen, 0);

    if (sendResult == -1) {

        fprintf(stderr, "Failed to send data to client .\n");
        socket_close((int)socket);
        return -1;
    }

    return sendResult;
}

void robotActiveWait(int numOfSteps)
{
    for (int i = 0; i < numOfSteps; i++) {
        if (robot->step(mTimeStep) == -1)
            break;
    }
}