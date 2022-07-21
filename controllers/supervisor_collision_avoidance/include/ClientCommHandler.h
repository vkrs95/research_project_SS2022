/* general imports */
#include <list>
#include <iostream>
#include <thread>
#include <mutex>

/* member imports */
#include "Message.h"

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

class ClientCommHandler
{
public:
	ClientCommHandler(int clientSocket);
	void addMsgToOutbox(Message* message);
	Message* getInboxMessage(void);
	std::string getClientName(void);

private:
	void socketCommunicationHandlerRoutine(void);
	void receiveMessage(void);
	void sendMessage(void);
	void sendACKMessage(void);
	void registerName(std::string name);

	std::string mClientName;
	int mClientSocket;
	std::list<Message> mMsgInbox;
	std::list<Message> mMsgOutbox;
	std::mutex msgMutex;
	std::thread* commHandlingThread;
};



