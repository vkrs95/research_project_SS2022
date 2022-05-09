/* general imports */
#include <vector>
#include <iostream>
#include <thread>

/* wifi imports */
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

#pragma once

class CommModuleTCPSocketServer{
public:
	CommModuleTCPSocketServer(int port = 1000);
	~CommModuleTCPSocketServer();
	int		socketSend(SOCKET socket, char* sendBuffer, int sendBufferLen);

protected:
	int		socketAccept(int server_fd);

private:
	bool	socketInit(void);
	int		createSocketServer(int port);
	bool	socketSetNonBlocking(int fd);
	bool	socketClose(int fd);
	bool	socketCleanup(void);
	void	socketListenerRoutine(void);

	int mListenSocket;
	int mClientSocket;
	int mServerPort = 0;
	unsigned int mClientCount = 0;
	std::vector<int> clientList;
	std::thread* listenerThread;
};



