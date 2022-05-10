/* general imports */
#include <vector>
#include <list>
#include <iostream>
#include <thread>
#include <mutex>

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
	void sendMessageToClient(std::string clientName, char* message);

	enum MessageType
	{
		REGISTER = 0, 
		UNREGISTER, 
		COLLISION
	};

protected:
	int socketAccept(int server_fd);
	const static int maxMsgLen = 512;

private:

	/* nested thread class */
	class ClientCommHandlerThread :
		public std::thread
	{
	public:
		ClientCommHandlerThread(int clientSocket);
		bool addMsgToOutbox(char* message);
		char* getInboxMessage(void);
		std::string getClientName(void);

	private:
		void socketCommunicationHandlerRoutine(void);
		void receiveMessage(void);
		void sendMessage(void);
		std::string extractNameFromMsg(char* message, int msgSize);

		std::string mClientName;
		int mClientSocket;
		std::list<char*> mMsgInbox;
		std::list<char*> mMsgOutbox;
		std::mutex msgMutex;
		std::thread* commHandlingThread;
	};


	/* internal functions */
	bool	socketInit(void);
	int		createSocketServer(int port);
	bool	socketSetNonBlocking(int fd);
	bool	socketClose(int fd);
	bool	socketCleanup(void);
	void	socketListenerRoutine(void);


	/* member variables */
	int mListenSocket;
	int mClientSocket;
	int mServerPort = 0;
	unsigned int mClientCount = 0;
	std::thread* listenerThread;
	std::vector<ClientCommHandlerThread*> mCommHandlerThreadPool;

};



