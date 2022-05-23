/* general imports */
#include <vector>
#include <list>
#include <map>
#include <iostream>
#include <thread>
#include <mutex>
#include <string>

/* member imports */
#include "ClientCommHandler.h"

/* tcp socket imports */
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

using coordinate = std::tuple<int, int>;

struct CollisionNotification {
	std::string clientName;
	std::tuple<int, int> startNode;
	std::tuple<int, int> goalNode;
	std::tuple<int, int> collisionNode;
};

class CommModuleTCPSocketServer{
public:
	CommModuleTCPSocketServer(int port = 1000);
	~CommModuleTCPSocketServer();
	void sendMessageToClient(std::string clientName, Message* message);
	bool checkCollisionNotifications(std::list<CollisionNotification>* collisionNotifications);
	void sendCollisionMessageReply(std::map<std::string, std::vector<coordinate>> clientPaths);

protected:
	int socketAccept(int server_fd);
	const static int maxMsgLen = 512;

private:

	/* internal functions */
	bool	socketInit(void);
	int		createSocketServer(int port);
	bool	socketSetNonBlocking(int fd);
	bool	socketClose(int fd);
	bool	socketCleanup(void);
	void	socketListenerRoutine(void);
	coordinate getCoordinatesTuple(std::string tupleString);
	std::string buildPathMsgString(std::vector<coordinate> path);


	/* member variables */
	int mListenSocket;
	int mClientSocket;
	int mServerPort = 0;
	unsigned int mClientCount = 0;
	std::thread* listenerThread;
	std::vector<ClientCommHandler*> mCommHandlerThreadPool;

};



