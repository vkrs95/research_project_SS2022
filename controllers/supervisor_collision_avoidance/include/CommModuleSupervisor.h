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

struct PathNotification {
	std::string clientName;
	std::tuple<int, int> startNode;
	std::tuple<int, int> goalNode;
};

struct CollisionNotification {
	std::string clientName;
	std::tuple<int, int> startNode;
	std::tuple<int, int> goalNode;
	std::tuple<int, int> collisionNode;
};

class CommModuleTCPSocketServer{

public:
	/*************************************
	* 
	*	Nested class PathNotification
	* 
	**************************************/
	class PathNotification {
	public:
		std::string clientName;
		std::tuple<int, int> startNode;
		std::tuple<int, int> goalNode;

		PathNotification(std::string clientName, std::string pathMsg);
	};

	/*************************************
	*
	*	Nested class PathNotification
	*
	**************************************/
	class CollisionNotification {
	public:
		std::string clientName;
		std::tuple<int, int> startNode;
		std::tuple<int, int> goalNode;
		std::tuple<int, int> collisionNode;

		CollisionNotification(std::string clientName, std::string collisionMsg);
	};


	CommModuleTCPSocketServer(int port = 1000);
	~CommModuleTCPSocketServer();
	bool checkClientNotifications(std::list<PathNotification>* pathNotifications, std::list<CollisionNotification>* collisionNotifications);
	void sendCollisionMessageReply(std::map<std::string, std::pair<int, std::vector<coordinate>>> clientPaths);
	void sendPathMessage(std::string clientName, std::vector<coordinate> clientPath);

protected:
	int socketAccept(int server_fd);
	static coordinate getCoordinatesTuple(std::string tupleString);
	const static int maxMsgLen = 512;

private:

	/* internal functions */
	bool	socketInit(void);
	int		createSocketServer(int port);
	bool	socketSetNonBlocking(int fd);
	bool	socketClose(int fd);
	bool	socketCleanup(void);
	void	socketListenerRoutine(void);
	void	sendMessageToClient(std::string clientName, Message* message);
	std::string buildPathMsgString(std::vector<coordinate> path);



	/* member variables */
	int mListenSocket;
	int mClientSocket;
	int mServerPort = 0;
	unsigned int mClientCount = 0;
	std::thread* listenerThread;
	std::vector<ClientCommHandler*> mCommHandlerThreadPool;

};



