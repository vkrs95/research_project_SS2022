#include <string>
#include <sstream>
#include <vector>
#pragma once

enum class MessageType
{
	REGISTER = 0,
	UNREGISTER,
	COLLISION, 
	INVALID = 99
};

class Message
{
public:
	/* constructors */
	Message(const char* data, int dataSize);
	Message(std::string data);
	
	/* get data functions */
	std::string getPayload(void);
	MessageType getType();
	int getSize(void);
	const char* getMessageAsChar(void);

	/* constants */
	static const int MAX_MSG_LEN = 512;

private:
	void parseMessage(const char* data);
	MessageType determineMessageType(int typeInt);

	MessageType mMsgType;
	std::string mPayload;
	int mMsgSize;
};



