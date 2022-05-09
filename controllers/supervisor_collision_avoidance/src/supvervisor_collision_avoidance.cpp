#include "supvervisor_collision_avoidance.h"


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
    struct sockaddr_in client;
    struct hostent* client_info;

#ifndef _WIN32
    socklen_t asize;
#else
    int asize;
#endif
    asize = sizeof(struct sockaddr_in);
    cfd = accept(server_fd, (struct sockaddr*)&client, &asize);

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

    client_info = gethostbyname((char*)inet_ntoa(client.sin_addr));
    printf("Accepted connection from: %s.\n", client_info->h_name);

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

    sfd = socket(AF_INET, SOCK_STREAM, 0); // IPPROTO_TCP instead of 0 ?

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

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    robot = new Robot();
    emitter = robot->getEmitter("emitter");

    /*************************************/
    unsigned int timeStep = robot->getBasicTimeStep();


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
    SOCKET m_socket;

    /* open a server on port 1000 */
    m_socket = create_socket_server(port);

    /* try to set socket non blocking */
    if (!socket_set_non_blocking(m_socket)) {
        /* failed to set server non-blocking */
        return -1;
    }

    printf("Waiting for a connection on port %d...\n", port);
    printf("SFD is %d...\n", (int) m_socket);

    while (robot->step(timeStep) != -1) {
        emitter->send(&m_socket, sizeof(m_socket));
    }

    /*  optional cleanup  */
    delete robot;

    return 0;
}