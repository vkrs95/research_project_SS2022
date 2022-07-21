#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#pragma once

enum class MessageType
{
	REGISTER = 0,
	UNREGISTER,
	PATH,
	COLLISION, 
	INVALID = 99
};

enum class MessageErrorCode
{
	SUCCESS = 0,
	ERROR_COLLISION_EVENT_CANCELED,
	INVALID = 99
};

class Message
{
public:
	/* constructors */
	Message(const char* data, int dataSize);
	Message(std::string data);
	Message(MessageType type, std::string payload);
	Message(MessageType type, MessageErrorCode errCode, std::string payload);
	Message(MessageType type, int errCode, std::string payload);
	
	/* get data functions */
	std::string getPayload(void);
	MessageType getType();
	MessageErrorCode getErrorCode();
	int getSize(void);
	const char* getMessageAsChar(void);

	/* constants */
	static const int MAX_MSG_LEN = 512;
	static const int MIN_MSG_LEN = 5;

private:
	void parseMessage(const char* data);

	MessageType mMsgType;
	MessageErrorCode mErrCode;
	std::string mPayload;
	int mMsgSize;
};



